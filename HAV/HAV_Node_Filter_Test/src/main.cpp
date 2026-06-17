#include <Arduino.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "hav_coefficients.h"

// =======================================================
// ADXL345 Register
// =======================================================
#define ADXL345_ADDR_1       0x53
#define ADXL345_ADDR_2       0x1D

#define REG_DEVID            0x00
#define REG_BW_RATE          0x2C
#define REG_POWER_CTL        0x2D
#define REG_DATA_FORMAT      0x31
#define REG_DATAX0           0x32

#define ADXL345_DEVID_VALUE  0xE5

// ADXL345 config
#define ADXL_BW_3200HZ       0x0F
#define ADXL_FORMAT_FULLRES_16G 0x0B

#define ADXL_SCALE_G_PER_LSB 0.0039f
#define G_TO_MPS2            9.80665f

// =======================================================
// I2C Pin
// =======================================================
#define PIN_I2C_SDA          21
#define PIN_I2C_SCL          22

// =======================================================
// BLE UUID
// Main Unit nanti subscribe ke characteristic ini
// =======================================================
#define BLE_DEVICE_NAME      "HAV_NODE"
#define BLE_SERVICE_UUID     "9b6f0001-5f5a-4f0d-9d7f-000000000001"
#define BLE_CHAR_UUID_HAV    "9b6f0002-5f5a-4f0d-9d7f-000000000002"

// =======================================================
// Sampling HAV
// =======================================================
const float FS = FS_HAV;   // 3200 Hz dari hav_coefficients.h

// 1 / 3200 = 312.5 us
// Pakai x100 supaya bisa represent 312.5 us sebagai 31250
const uint32_t SAMPLE_PERIOD_US_X100 = 31250;
const uint16_t HAV_EPOCH_SAMPLES = 3200;

// =======================================================
// Global state
// =======================================================
uint8_t adxlAddress = 0x00;
bool adxlAvailable = false;

BLECharacteristic *havCharacteristic = nullptr;
bool bleClientConnected = false;

uint32_t packetCounter = 0;

// =======================================================
// Biquad Cascade Filter
// Format coeff: {b0, b1, b2, a1, a2}
// =======================================================
class BiquadCascade {
public:
  static const int MAX_SECTIONS = 4;

  BiquadCascade(const float (*coeff)[5], int sections) {
    this->coeff = coeff;
    this->sections = sections;
    reset();
  }

  float process(float input) {
    float x = input;

    for (int i = 0; i < sections; i++) {
      float b0 = coeff[i][0];
      float b1 = coeff[i][1];
      float b2 = coeff[i][2];
      float a1 = coeff[i][3];
      float a2 = coeff[i][4];

      float y = b0 * x + b1 * x1[i] + b2 * x2[i]
              - a1 * y1[i] - a2 * y2[i];

      x2[i] = x1[i];
      x1[i] = x;

      y2[i] = y1[i];
      y1[i] = y;

      x = y;
    }

    return x;
  }

  void reset() {
    for (int i = 0; i < MAX_SECTIONS; i++) {
      x1[i] = 0.0f;
      x2[i] = 0.0f;
      y1[i] = 0.0f;
      y2[i] = 0.0f;
    }
  }

private:
  const float (*coeff)[5];
  int sections;

  float x1[MAX_SECTIONS];
  float x2[MAX_SECTIONS];
  float y1[MAX_SECTIONS];
  float y2[MAX_SECTIONS];
};

// Wh untuk semua sumbu HAV
BiquadCascade filterX(coeff_wh, NUM_SECTIONS_WH);
BiquadCascade filterY(coeff_wh, NUM_SECTIONS_WH);
BiquadCascade filterZ(coeff_wh, NUM_SECTIONS_WH);

// =======================================================
// I2C Helper
// =======================================================
bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t n = Wire.requestFrom((int)addr, 1);

  if (n != 1 || Wire.available() < 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

bool readMultipleRegisters(uint8_t addr, uint8_t startReg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t n = Wire.requestFrom((int)addr, (int)length);

  if (n != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    if (!Wire.available()) {
      return false;
    }
    buffer[i] = Wire.read();
  }

  return true;
}

// =======================================================
// ADXL345 Detect + Init
// =======================================================
bool detectADXL345() {
  uint8_t devid = 0;

  if (readRegister(ADXL345_ADDR_1, REG_DEVID, devid)) {
    if (devid == ADXL345_DEVID_VALUE) {
      adxlAddress = ADXL345_ADDR_1;
      return true;
    }
  }

  if (readRegister(ADXL345_ADDR_2, REG_DEVID, devid)) {
    if (devid == ADXL345_DEVID_VALUE) {
      adxlAddress = ADXL345_ADDR_2;
      return true;
    }
  }

  adxlAddress = 0x00;
  return false;
}

bool setupADXL345() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);

  if (!detectADXL345()) {
    return false;
  }

  Serial.print("ADXL345 HAV terdeteksi di address 0x");
  Serial.println(adxlAddress, HEX);

  // Standby
  if (!writeRegister(adxlAddress, REG_POWER_CTL, 0x00)) {
    return false;
  }

  // Output data rate 3200 Hz
  if (!writeRegister(adxlAddress, REG_BW_RATE, ADXL_BW_3200HZ)) {
    return false;
  }

  // Full resolution, range +/-16g
  if (!writeRegister(adxlAddress, REG_DATA_FORMAT, ADXL_FORMAT_FULLRES_16G)) {
    return false;
  }

  // Measurement mode
  if (!writeRegister(adxlAddress, REG_POWER_CTL, 0x08)) {
    return false;
  }

  Wire.setClock(400000);

  Serial.println("ADXL345 HAV setup selesai.");
  Serial.println("Range    : +/-16g full resolution");
  Serial.println("DataRate : 3200 Hz");

  return true;
}

bool readADXL345(float &ax, float &ay, float &az) {
  if (!adxlAvailable) {
    return false;
  }

  uint8_t data[6];

  if (!readMultipleRegisters(adxlAddress, REG_DATAX0, data, 6)) {
    return false;
  }

  int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
  int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
  int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);

  ax = rawX * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
  ay = rawY * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
  az = rawZ * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;

  return true;
}

// =======================================================
// BLE Callback
// =======================================================
class HavBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleClientConnected = true;
    Serial.println("BLE client connected.");
  }

  void onDisconnect(BLEServer *server) override {
    bleClientConnected = false;
    Serial.println("BLE client disconnected. Restart advertising.");
    server->getAdvertising()->start();
  }
};

// =======================================================
// BLE Setup
// =======================================================
void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new HavBleServerCallbacks());

  BLEService *service = server->createService(BLE_SERVICE_UUID);

  havCharacteristic = service->createCharacteristic(
    BLE_CHAR_UUID_HAV,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  havCharacteristic->addDescriptor(new BLE2902());

  // Initial value
  havCharacteristic->setValue("HAV,0,0,0,0,0,0,0");

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  advertising->start();

  Serial.println("BLE HAV Node advertising started.");
}

// =======================================================
// Send HAV packet
// Format:
// HAV,seq,millis_ms,ahwx,ahwy,ahwz,ahv,n_samples
// =======================================================
void sendHavBlePacket(float ahwx, float ahwy, float ahwz, float ahv, uint16_t nSamples) {
  char payload[128];

  uint32_t nowMs = millis();

  snprintf(payload, sizeof(payload),
           "HAV,%lu,%lu,%.6f,%.6f,%.6f,%.6f,%u",
           packetCounter,
           nowMs,
           ahwx,
           ahwy,
           ahwz,
           ahv,
           nSamples);

  // Serial tetap print meskipun belum connect BLE
  Serial.println(payload);

  if (havCharacteristic != nullptr) {
    havCharacteristic->setValue(payload);

    if (bleClientConnected) {
      havCharacteristic->notify();
    }
  }

  packetCounter++;
}

// =======================================================
// Setup
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("HAV Node ESP32");
  Serial.println("ADXL345 + Wh Filter + BLE TX");
  Serial.println("No RTC, no SD, no OLED");
  Serial.println("=================================");

  adxlAvailable = setupADXL345();

  if (!adxlAvailable) {
    Serial.println("ERROR: ADXL345 HAV tidak terdeteksi.");
    Serial.println("Cek wiring:");
    Serial.println("VCC -> 3V3");
    Serial.println("GND -> GND");
    Serial.println("SDA -> GPIO21");
    Serial.println("SCL -> GPIO22");
    Serial.println("CS  -> 3V3");
    Serial.println("SDO -> GND atau 3V3");
  }

  setupBLE();

  Serial.print("Sampling rate HAV = ");
  Serial.print(FS);
  Serial.println(" Hz");

  Serial.println("Payload BLE:");
  Serial.println("HAV,seq,millis_ms,ahwx,ahwy,ahwz,ahv,n_samples");
}

// =======================================================
// Loop
// =======================================================
void loop() {
  static uint64_t nextSampleTimeUsX100 =
    (uint64_t)micros() * 100ULL + SAMPLE_PERIOD_US_X100;

  static float sumX2 = 0.0f;
  static float sumY2 = 0.0f;
  static float sumZ2 = 0.0f;
  static uint16_t sampleCount = 0;

  static float lastRawX = 0.0f;
  static float lastRawY = 0.0f;
  static float lastRawZ = 0.0f;

  static uint32_t lastErrorPrintMs = 0;

  uint64_t nowUsX100 = (uint64_t)micros() * 100ULL;

  if (nowUsX100 >= nextSampleTimeUsX100) {
    nextSampleTimeUsX100 += SAMPLE_PERIOD_US_X100;

    float ax, ay, az;

    if (!readADXL345(ax, ay, az)) {
      if (millis() - lastErrorPrintMs >= 1000) {
        lastErrorPrintMs = millis();
        Serial.println("ADXL345 HAV tidak terbaca. Sampel dilewati.");

        Wire.setClock(100000);
        adxlAvailable = setupADXL345();
        Wire.setClock(400000);
      }

      return;
    }

    lastRawX = ax;
    lastRawY = ay;
    lastRawZ = az;

    // Wh filter untuk semua sumbu HAV
    float axWh = filterX.process(ax);
    float ayWh = filterY.process(ay);
    float azWh = filterZ.process(az);

    sumX2 += axWh * axWh;
    sumY2 += ayWh * ayWh;
    sumZ2 += azWh * azWh;
    sampleCount++;

    // Per 1 detik: 3200 sampel
    if (sampleCount >= HAV_EPOCH_SAMPLES) {
      float ahwx = sqrtf(sumX2 / sampleCount);
      float ahwy = sqrtf(sumY2 / sampleCount);
      float ahwz = sqrtf(sumZ2 / sampleCount);

      float ahv = sqrtf(
        ahwx * ahwx +
        ahwy * ahwy +
        ahwz * ahwz
      );

      Serial.print("RAW: ");
      Serial.print("ax=");
      Serial.print(lastRawX, 6);
      Serial.print(", ay=");
      Serial.print(lastRawY, 6);
      Serial.print(", az=");
      Serial.print(lastRawZ, 6);
      Serial.print(" | ");

      sendHavBlePacket(ahwx, ahwy, ahwz, ahv, sampleCount);

      sumX2 = 0.0f;
      sumY2 = 0.0f;
      sumZ2 = 0.0f;
      sampleCount = 0;
    }
  }
}