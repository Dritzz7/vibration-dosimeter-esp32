#include <Arduino.h>
#include <Wire.h>
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

// ADXL345 detected address
uint8_t adxlAddress = 0x00;
bool adxlAvailable = false;

// =======================================================
// Biquad Cascade Filter
// Format coeff: {b0, b1, b2, a1, a2}
// =======================================================
class BiquadCascade {
public:
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
  static const int MAX_SECTIONS = 4;

  const float (*coeff)[5];
  int sections;

  float x1[MAX_SECTIONS];
  float x2[MAX_SECTIONS];
  float y1[MAX_SECTIONS];
  float y2[MAX_SECTIONS];
};

// =======================================================
// Filter Wh untuk HAV
// =======================================================
BiquadCascade filterX(coeff_wh, NUM_SECTIONS_WH);
BiquadCascade filterY(coeff_wh, NUM_SECTIONS_WH);
BiquadCascade filterZ(coeff_wh, NUM_SECTIONS_WH);

// =======================================================
// Sampling
// =======================================================
const float FS = FS_HAV;   // 3200 Hz dari hav_coefficients.h

// 1/3200 s = 312.5 us
// Supaya 312.5 us bisa presisi, pakai skala x100
const uint32_t SAMPLE_PERIOD_US_X100 = 31250;

uint64_t nextSampleTimeUsX100 = 0;

// RMS accumulator
float sumX2 = 0.0f;
float sumY2 = 0.0f;
float sumZ2 = 0.0f;

uint16_t sampleCount = 0;

// Print limiter
uint32_t lastRawPrintMillis = 0;
uint32_t lastErrorPrintMillis = 0;

// =======================================================
// I2C helper functions
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
// Scan ADXL345 address
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

// =======================================================
// Init ADXL345
// =======================================================
bool setupADXL345() {
  Wire.begin(21, 22);       // SDA = GPIO21, SCL = GPIO22
  Wire.setClock(100000);    // 100 kHz dulu untuk deteksi

  if (!detectADXL345()) {
    return false;
  }

  Serial.print("ADXL345 terdeteksi di address 0x");
  Serial.println(adxlAddress, HEX);

  // Standby dulu
  if (!writeRegister(adxlAddress, REG_POWER_CTL, 0x00)) {
    return false;
  }

  // Data rate 3200 Hz
  // BW_RATE = 0x0F untuk 3200 Hz
  if (!writeRegister(adxlAddress, REG_BW_RATE, 0x0F)) {
    return false;
  }

  // DATA_FORMAT:
  // bit 3 FULL_RES = 1
  // range +/-16g = 0x03
  // 0x08 | 0x03 = 0x0B
  if (!writeRegister(adxlAddress, REG_DATA_FORMAT, 0x0B)) {
    return false;
  }

  // Measurement mode
  if (!writeRegister(adxlAddress, REG_POWER_CTL, 0x08)) {
    return false;
  }

  // Naikkan speed I2C setelah sensor terdeteksi
  Wire.setClock(400000);

  Serial.println("ADXL345 setup selesai.");
  Serial.println("Range    : +/-16g full resolution");
  Serial.println("DataRate : 3200 Hz");

  return true;
}

// =======================================================
// Read ADXL345 raw acceleration
// Output: m/s^2
// =======================================================
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

  // ADXL345 full-resolution scale ≈ 3.9 mg/LSB
  const float SCALE_G_PER_LSB = 0.0039f;
  const float G_TO_MPS2 = 9.80665f;

  ax = rawX * SCALE_G_PER_LSB * G_TO_MPS2;
  ay = rawY * SCALE_G_PER_LSB * G_TO_MPS2;
  az = rawZ * SCALE_G_PER_LSB * G_TO_MPS2;

  return true;
}

// =======================================================
// Setup
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("HAV Node ADXL345 + Wh Filter");
  Serial.println("=================================");

  adxlAvailable = setupADXL345();

  if (!adxlAvailable) {
    Serial.println("ERROR: ADXL345 tidak terdeteksi.");
    Serial.println("Cek wiring:");
    Serial.println("VCC -> 3V3");
    Serial.println("GND -> GND");
    Serial.println("SDA -> GPIO21 / D21");
    Serial.println("SCL -> GPIO22 / D22");
    Serial.println("CS  -> 3V3");
    Serial.println("SDO -> GND atau 3V3");
    Serial.println();
    Serial.println("Program tetap jalan, tapi tidak akan memfilter data palsu.");
  }

  Serial.print("Sampling rate filter HAV = ");
  Serial.print(FS);
  Serial.println(" Hz");

  nextSampleTimeUsX100 = (uint64_t)micros() * 100ULL + SAMPLE_PERIOD_US_X100;
}

// =======================================================
// Loop
// =======================================================
void loop() {
  uint64_t nowUsX100 = (uint64_t)micros() * 100ULL;

  if (nowUsX100 >= nextSampleTimeUsX100) {
    nextSampleTimeUsX100 += SAMPLE_PERIOD_US_X100;

    float ax, ay, az;

    if (!readADXL345(ax, ay, az)) {
      if (millis() - lastErrorPrintMillis >= 1000) {
        lastErrorPrintMillis = millis();

        Serial.println("ADXL345 tidak terbaca. Sampel dilewati.");

        // Coba deteksi ulang kalau sensor baru dipasang
        Wire.setClock(100000);
        adxlAvailable = setupADXL345();
        Wire.setClock(400000);
      }

      return;
    }

    // Print raw data 1 detik sekali
    if (millis() - lastRawPrintMillis >= 1000) {
      lastRawPrintMillis = millis();

      Serial.print("RAW ax=");
      Serial.print(ax, 6);
      Serial.print(", ay=");
      Serial.print(ay, 6);
      Serial.print(", az=");
      Serial.println(az, 6);
    }

    // Masuk filter Wh
    float axWh = filterX.process(ax);
    float ayWh = filterY.process(ay);
    float azWh = filterZ.process(az);

    // Akumulasi RMS
    sumX2 += axWh * axWh;
    sumY2 += ayWh * ayWh;
    sumZ2 += azWh * azWh;

    sampleCount++;

    if (sampleCount >= 3200) {
      float ahwx = sqrtf(sumX2 / sampleCount);
      float ahwy = sqrtf(sumY2 / sampleCount);
      float ahwz = sqrtf(sumZ2 / sampleCount);

      float ahv = sqrtf(
        ahwx * ahwx +
        ahwy * ahwy +
        ahwz * ahwz
      );

      Serial.print("HAV RMS: ");
      Serial.print("ahwx=");
      Serial.print(ahwx, 6);
      Serial.print(", ahwy=");
      Serial.print(ahwy, 6);
      Serial.print(", ahwz=");
      Serial.print(ahwz, 6);
      Serial.print(", ahv=");
      Serial.println(ahv, 6);

      sumX2 = 0.0f;
      sumY2 = 0.0f;
      sumZ2 = 0.0f;
      sampleCount = 0;
    }
  }
}