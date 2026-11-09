#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// =============================================================================
// SPI HARDWARE PIN DEFINITIONS (ESP32 VSPI / HAV-Node Hardware)
// =============================================================================
// Wire connections:
//   - VCC / VS   -> 3.3V
//   - GND        -> GND
//   - CS         -> GPIO 25 (or GPIO 5 on standard VSPI)
//   - SCK / SCL  -> GPIO 18
//   - MOSI / SDA -> GPIO 23
//   - MISO / SDO -> GPIO 19
//   - INT1       -> GPIO 35 (HAV Node PCB, or GPIO 34 for Main Unit)
#define PIN_SPI_SCK   18    ///< SCK / SCL
#define PIN_SPI_MISO  19    ///< MISO / SDO
#define PIN_SPI_MOSI  23    ///< MOSI / SDA
#define PIN_ADXL_CS   25    ///< CS (GPIO25 on HAV Node PCB, change to 5 if using GPIO5)
#define PIN_ADXL_INT1 35    ///< INT1 Data-Ready interrupt pin (GPIO35 on HAV Node, GPIO34 on Main Unit)

/* Assign a unique ID to this sensor in 4-wire SPI mode */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(
    PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_ADXL_CS, 12345
);

// Flag dan counter interrupt data ready
volatile bool dataReadyFlag = false;
volatile uint32_t isrCounter = 0;

/**
 * @brief Interrupt Service Routine (ISR) yang dipicu saat ADXL345 menyelesaikan konversi data
 */
void IRAM_ATTR isrADXLDataReady() {
  dataReadyFlag = true;
  isrCounter++;
}

/**
 * @brief Helper untuk menulis 1 byte ke register ADXL345 via 4-wire SPI secara langsung
 */
void writeRegisterSPI(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ADXL_CS, LOW);
  SPI.transfer(reg & 0x3F); // R/W = 0, MB = 0
  SPI.transfer(value);
  digitalWrite(PIN_ADXL_CS, HIGH);
  SPI.endTransaction();
}

/**
 * @brief Helper untuk membaca 1 byte dari register ADXL345 via SPI
 */
uint8_t readRegisterSPI(uint8_t reg) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ADXL_CS, LOW);
  SPI.transfer((reg & 0x3F) | 0x80); // R/W = 1, MB = 0
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(PIN_ADXL_CS, HIGH);
  SPI.endTransaction();
  return val;
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print ("Sensor:     "); Serial.println(sensor.name);
  Serial.print ("Driver Ver: "); Serial.println(sensor.version);
  Serial.print ("Unique ID:  "); Serial.println(sensor.sensor_id);
  Serial.print ("Device ID:  0x"); Serial.println(accel.getDeviceID(), HEX);
  Serial.print ("Max Value:  "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print ("Min Value:  "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print ("Resolution: "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print ("Data Rate: ");

  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print ("3200 ");
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print ("1600 ");
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print ("800 ");
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print ("400 ");
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print ("200 ");
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print ("100 ");
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print ("50 ");
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print ("25 ");
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print ("12.5 ");
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print ("6.25 ");
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print ("3.13 ");
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print ("1.56 ");
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print ("0.78 ");
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print ("0.39 ");
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print ("0.20 ");
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print ("0.10 ");
      break;
    default:
      Serial.print ("???? ");
      break;
  }
  Serial.println(" Hz");
}

void displayRange(void)
{
  Serial.print ("Range: +/- ");

  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print ("16 ");
      break;
    case ADXL345_RANGE_8_G:
      Serial.print ("8 ");
      break;
    case ADXL345_RANGE_4_G:
      Serial.print ("4 ");
      break;
    case ADXL345_RANGE_2_G:
      Serial.print ("2 ");
      break;
    default:
      Serial.print ("?? ");
      break;
  }
  Serial.println(" g");
}

void setup(void)
{
  Serial.begin(9600);
  delay(500);

  Serial.println("==========================================");
  Serial.println("  ADXL345 Accelerometer Test (SPI Mode)   ");
  Serial.println("==========================================");
  Serial.printf("SPI Pins Configured:\n");
  Serial.printf("  - SCK  : GPIO%d\n", PIN_SPI_SCK);
  Serial.printf("  - MISO : GPIO%d\n", PIN_SPI_MISO);
  Serial.printf("  - MOSI : GPIO%d\n", PIN_SPI_MOSI);
  Serial.printf("  - CS   : GPIO%d\n\n", PIN_ADXL_CS);

  /* Initialise the sensor over 4-wire SPI */
  if (!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    Serial.println("Troubleshooting Checklist:");
    Serial.println("  1. VCC / VS  -> 3.3V (do NOT use 5V unless module has an onboard regulator)");
    Serial.println("  2. GND       -> GND");
    Serial.printf ("  3. CS        -> GPIO%d\n", PIN_ADXL_CS);
    Serial.printf ("  4. SCL/SCK   -> GPIO%d\n", PIN_SPI_SCK);
    Serial.printf ("  5. SDA/MOSI  -> GPIO%d\n", PIN_SPI_MOSI);
    Serial.printf ("  6. SDO/MISO  -> GPIO%d\n", PIN_SPI_MISO);
    Serial.println("  7. Check if CS has a 10k pull-up resistor to 3.3V");
    while (1) {
      delay(1000);
    }
  }

  /* Set the range to +/- 16g (standard for vibration dosimetry) */
  accel.setRange(ADXL345_RANGE_16_G);

  // Set ODR (Output Data Rate) - misalnya 100 Hz atau 400 Hz untuk pengujian
  accel.setDataRate(ADXL345_DATARATE_100_HZ);

  // ===========================================================================
  // KONFIGURASI HARDWARE INTERRUPT (INT1 - DATA READY)
  // ===========================================================================
  // 1. Map DATA_READY interrupt ke pin INT1 (Bit 7 = 0 pada REG_INT_MAP 0x2F)
  writeRegisterSPI(ADXL345_REG_INT_MAP, 0x00);

  // 2. Aktifkan DATA_READY interrupt (Bit 7 = 1 pada REG_INT_ENABLE 0x2E -> 0x80)
  writeRegisterSPI(ADXL345_REG_INT_ENABLE, 0x80);

  // 3. Konfigurasi GPIO ESP32 sebagai input interrupt
  pinMode(PIN_ADXL_INT1, INPUT);

  // 4. Bersihkan pending interrupt dengan membaca INT_SOURCE sekali
  readRegisterSPI(ADXL345_REG_INT_SOURCE);

  // 5. Hubungkan ISR dengan trigger RISING (default active-HIGH pada ADXL345)
  attachInterrupt(digitalPinToInterrupt(PIN_ADXL_INT1), isrADXLDataReady, RISING);

  Serial.printf("Hardware Interrupt INT1 Diaktifkan pada GPIO%d (RISING, DATA_READY)\n\n", PIN_ADXL_INT1);

  /* Display basic information on this sensor */
  displaySensorDetails();

  /* Display data rate and range */
  displayDataRate();
  displayRange();
  Serial.println("Menunggu trigger interrupt INT1 dari ADXL345...\n");
}

void loop(void)
{
  // Cek apakah interrupt hardware INT1 telah memicu flag dataReady
  if (dataReadyFlag) {
    dataReadyFlag = false;

    /* Dapatkan pembacaan akselerasi terbaru */
    sensors_event_t event;
    accel.getEvent(&event);

    /* Bersihkan status interrupt ADXL345 agar pulsa INT1 siap untuk siklus berikutnya */
    readRegisterSPI(ADXL345_REG_INT_SOURCE);

    // Tampilkan data setiap kelipatan tertentu agar serial monitor tidak flood
    static uint32_t lastPrintMillis = 0;
    static uint32_t sampleCount = 0;
    sampleCount++;

    if (millis() - lastPrintMillis >= 500) {
      lastPrintMillis = millis();

      Serial.printf("[INT1 Triggered #%lu] ", (unsigned long)isrCounter);
      Serial.print("X: "); Serial.print(event.acceleration.x, 3); Serial.print("  ");
      Serial.print("Y: "); Serial.print(event.acceleration.y, 3); Serial.print("  ");
      Serial.print("Z: "); Serial.print(event.acceleration.z, 3); Serial.print(" m/s^2");
      Serial.printf(" | Sample count: %lu\n", (unsigned long)sampleCount);
    }
  }
}

