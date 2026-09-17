/**
 * @file    main.cpp
 * @brief   ADXL345 High-Speed Raw Sensor Visualizer & Validation Firmware
 *
 * @details Developed for validating sensor data integrity and dynamic response
 *          under various physical excitation inputs:
 *            1. Static 1g Earth gravity test (orientation & sensitivity check)
 *            2. Transient impulse / shock response (tap, strike, drop)
 *            3. Periodic sinusoidal vibration response (harmonic excitation / motor)
 *
 *          Features:
 *            - Microsecond deterministic sampling timer (no jitter)
 *            - Full-resolution ±16g range (3.9 mg/LSB sensitivity)
 *            - 400 kHz Fast I2C burst reads (6 bytes single-transaction)
 *            - Configurable ODR: 100 Hz, 200 Hz (WBV), 400 Hz, 800 Hz, 1600 Hz, 3200 Hz (HAV)
 *            - Arduino Serial Plotter & Teleplot compatible output format
 *            - Interactive CLI commands (ODR switch, Tare offset, Unit toggle, Stats)
 *            - Real-time sampling frequency ($F_{actual}$) telemetry
 *
 * @author  Team EL4060
 * @date    2026-09-16
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =============================================================================
// HARDWARE PIN DEFINITIONS & I2C REGISTERS
// =============================================================================
#define PIN_I2C_SDA          21
#define PIN_I2C_SCL          22

#define ADXL345_ADDR_PRIMARY   0x53
#define ADXL345_ADDR_ALT       0x1D

#define REG_DEVID            0x00
#define REG_BW_RATE          0x2C
#define REG_POWER_CTL        0x2D
#define REG_DATA_FORMAT      0x31
#define REG_DATAX0           0x32

#define ADXL345_DEVID_VAL    0xE5

// ADXL345 ODR Register Values
#define BW_RATE_100HZ        0x0A
#define BW_RATE_200HZ        0x0B
#define BW_RATE_400HZ        0x0C
#define BW_RATE_800HZ        0x0D
#define BW_RATE_1600HZ       0x0E
#define BW_RATE_3200HZ       0x0F

// Format: FULL_RES | Range ±16g
#define DATA_FORMAT_FULLRES_16G 0x0B

// Calibration & Scale
#define ADXL_SCALE_G_PER_LSB 0.00390625f // 3.9 mg / LSB (1/256)
#define G_TO_MPS2            9.80665f    // Standard gravity (m/s²)

// =============================================================================
// GLOBAL CONFIGURATION & STATE
// =============================================================================
uint8_t  sensorAddress   = 0x00;
bool     sensorAvailable = false;
bool     streamingActive = true;
bool     unitIsMps2      = true;     // true: m/s², false: g

// Sampling rate control
uint16_t targetRateHz    = 200;      // Default 200 Hz (matched to WBV)
uint32_t periodMicros    = 5000;     // 1,000,000 / targetRateHz
uint32_t nextSampleMicros = 0;

// Decimation for Serial Plotter at ultra-high ODRs (e.g. 1600/3200 Hz)
uint8_t  decimationFactor = 1;
uint8_t  decimationCounter = 0;

// Tare / DC Offset compensation
float offsetAx = 0.0f;
float offsetAy = 0.0f;
float offsetAz = 0.0f;
bool  tareEnabled = false;

// Telemetry & frequency diagnostics
uint32_t sampleCountSec   = 0;
uint32_t lastStatsMillis  = 0;
float    actualRateHz     = 0.0f;
float    peakAx = 0.0f, peakAy = 0.0f, peakAz = 0.0f, peakMag = 0.0f;

// =============================================================================
// LOW-LEVEL I2C HELPER FUNCTIONS
// =============================================================================
bool writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool readRegister(uint8_t addr, uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr, 1) != 1) return false;
    val = Wire.read();
    return true;
}

bool readBurstData(uint8_t addr, int16_t &rawX, int16_t &rawY, int16_t &rawZ) {
    Wire.beginTransmission(addr);
    Wire.write(REG_DATAX0);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((int)addr, 6) != 6) {
        while (Wire.available()) Wire.read();
        return false;
    }

    uint8_t buf[6];
    for (int i = 0; i < 6; i++) {
        buf[i] = Wire.read();
    }

    rawX = (int16_t)((buf[1] << 8) | buf[0]);
    rawY = (int16_t)((buf[3] << 8) | buf[2]);
    rawZ = (int16_t)((buf[5] << 8) | buf[4]);
    return true;
}

// =============================================================================
// SENSOR SETUP & RATE SWITCHING
// =============================================================================
bool configureSensorODR(uint16_t rateHz) {
    uint8_t bwVal = BW_RATE_200HZ;
    switch (rateHz) {
        case 100:  bwVal = BW_RATE_100HZ;  decimationFactor = 1; break;
        case 200:  bwVal = BW_RATE_200HZ;  decimationFactor = 1; break;
        case 400:  bwVal = BW_RATE_400HZ;  decimationFactor = 1; break;
        case 800:  bwVal = BW_RATE_800HZ;  decimationFactor = 2; break; // Downsample 2x for 115200 baud
        case 1600: bwVal = BW_RATE_1600HZ; decimationFactor = 4; break; // Downsample 4x for 115200 baud
        case 3200: bwVal = BW_RATE_3200HZ; decimationFactor = 8; break; // Downsample 8x for 115200 baud
        default:
            Serial.printf("[ERR] Unsupported rate %d Hz! Valid: 100, 200, 400, 800, 1600, 3200\n", rateHz);
            return false;
    }

    // Set into Standby to modify BW_RATE
    writeRegister(sensorAddress, REG_POWER_CTL, 0x00);
    if (!writeRegister(sensorAddress, REG_BW_RATE, bwVal)) {
        return false;
    }
    // Return to Measurement Mode
    writeRegister(sensorAddress, REG_POWER_CTL, 0x08);

    targetRateHz = rateHz;
    periodMicros = 1000000UL / rateHz;
    decimationCounter = 0;

    Serial.printf("# [CFG] ODR set to %d Hz (Period = %u us, Decimation = %dx)\n",
                  targetRateHz, periodMicros, decimationFactor);
    return true;
}

bool initADXL345() {
    uint8_t devId = 0;

    // Scan address 0x53 (SDO=GND)
    if (readRegister(ADXL345_ADDR_PRIMARY, REG_DEVID, devId) && devId == ADXL345_DEVID_VAL) {
        sensorAddress = ADXL345_ADDR_PRIMARY;
    }
    // Scan address 0x1D (SDO=VCC)
    else if (readRegister(ADXL345_ADDR_ALT, REG_DEVID, devId) && devId == ADXL345_DEVID_VAL) {
        sensorAddress = ADXL345_ADDR_ALT;
    } else {
        Serial.println("[ERR] No ADXL345 detected! Check SDA/SCL wiring and VCC/GND.");
        return false;
    }

    Serial.printf("# [INIT] ADXL345 detected at I2C address 0x%02X (DEVID = 0x%02X)\n",
                  sensorAddress, devId);

    // Standby mode
    writeRegister(sensorAddress, REG_POWER_CTL, 0x00);
    // Full resolution ±16g
    writeRegister(sensorAddress, REG_DATA_FORMAT, DATA_FORMAT_FULLRES_16G);
    // Configure default rate (200 Hz)
    configureSensorODR(targetRateHz);
    // Measurement mode
    writeRegister(sensorAddress, REG_POWER_CTL, 0x08);

    sensorAvailable = true;
    return true;
}

// =============================================================================
// CALIBRATION / TARE ROUTINE
// =============================================================================
void performTareCalibration() {
    Serial.println("# [CAL] Calculating static zero-tare offset (keep sensor stationary for 1s)...");
    const int TARE_SAMPLES = 200;
    float sumX = 0, sumY = 0, sumZ = 0;
    int validCount = 0;

    for (int i = 0; i < TARE_SAMPLES; i++) {
        int16_t rx, ry, rz;
        if (readBurstData(sensorAddress, rx, ry, rz)) {
            float k = ADXL_SCALE_G_PER_LSB * (unitIsMps2 ? G_TO_MPS2 : 1.0f);
            sumX += rx * k;
            sumY += ry * k;
            sumZ += rz * k;
            validCount++;
        }
        delay(5);
    }

    if (validCount > 0) {
        offsetAx = sumX / validCount;
        offsetAy = sumY / validCount;
        offsetAz = sumZ / validCount;
        tareEnabled = true;
        Serial.printf("# [CAL] Tare set! Offset: X=%.3f, Y=%.3f, Z=%.3f (%s)\n",
                      offsetAx, offsetAy, offsetAz, unitIsMps2 ? "m/s^2" : "g");
    } else {
        Serial.println("# [CAL] Tare failed: sensor read error!");
    }
}

void resetTareCalibration() {
    offsetAx = 0.0f;
    offsetAy = 0.0f;
    offsetAz = 0.0f;
    tareEnabled = false;
    Serial.println("# [CAL] Tare reset to raw absolute (gravity included).");
}

// =============================================================================
// CLI COMMAND PARSER
// =============================================================================
void printHelp() {
    Serial.println("================================================================");
    Serial.println("   ADXL345 Vibration Visualizer — Interactive Command Menu      ");
    Serial.println("================================================================");
    Serial.println(" [1] Set ODR to 100 Hz");
    Serial.println(" [2] Set ODR to 200 Hz  (ISO 2631-1 WBV Standard)");
    Serial.println(" [3] Set ODR to 400 Hz");
    Serial.println(" [4] Set ODR to 800 Hz");
    Serial.println(" [5] Set ODR to 1600 Hz");
    Serial.println(" [6] Set ODR to 3200 Hz (ISO 5349-1 HAV Standard)");
    Serial.println(" [t] Tare Zero Offset   (Calibrate stationary DC offset)");
    Serial.println(" [r] Reset Tare Offset  (Show absolute 1g Earth gravity)");
    Serial.println(" [u] Toggle Units       (Switch between m/s^2 and g)");
    Serial.println(" [p] Pause / Resume Stream");
    Serial.println(" [s] Show Telemetry Status & Actual Frequency");
    Serial.println(" [h] Print this Help Menu");
    Serial.println("================================================================");
}

void printStatus() {
    Serial.println("----------------- SENSOR STATUS -----------------");
    Serial.printf(" I2C Address   : 0x%02X\n", sensorAddress);
    Serial.printf(" Target ODR    : %d Hz\n", targetRateHz);
    Serial.printf(" Actual Fs     : %.2f Hz\n", actualRateHz);
    Serial.printf(" Unit          : %s\n", unitIsMps2 ? "m/s^2" : "g");
    Serial.printf(" Tare State    : %s (X=%.3f, Y=%.3f, Z=%.3f)\n",
                  tareEnabled ? "ACTIVE (Zero-centered)" : "OFF (Raw 1g)",
                  offsetAx, offsetAy, offsetAz);
    Serial.printf(" Peak Since Sec: X=%.2f, Y=%.2f, Z=%.2f, Mag=%.2f\n",
                  peakAx, peakAy, peakAz, peakMag);
    Serial.println("-------------------------------------------------");
}

void handleSerialCommands() {
    if (!Serial.available()) return;
    char c = (char)Serial.read();

    switch (c) {
        case '1': configureSensorODR(100);  break;
        case '2': configureSensorODR(200);  break;
        case '3': configureSensorODR(400);  break;
        case '4': configureSensorODR(800);  break;
        case '5': configureSensorODR(1600); break;
        case '6': configureSensorODR(3200); break;
        case 't': case 'T': performTareCalibration(); break;
        case 'r': case 'R': resetTareCalibration();   break;
        case 'u': case 'U':
            unitIsMps2 = !unitIsMps2;
            Serial.printf("# [CFG] Unit toggled to: %s\n", unitIsMps2 ? "m/s^2" : "g");
            break;
        case 'p': case 'P':
            streamingActive = !streamingActive;
            Serial.printf("# [STREAM] %s\n", streamingActive ? "RESUMED" : "PAUSED");
            break;
        case 's': case 'S': printStatus(); break;
        case 'h': case 'H': case '?': printHelp(); break;
        default: break;
    }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================================");
    Serial.println("   Vibration Dosimeter ESP32 — Raw Sensor Visualizer   ");
    Serial.println("========================================================");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000); // 400 kHz Fast-Mode I2C

    if (!initADXL345()) {
        Serial.println("[CRITICAL] Sensor initialization failed! System halted.");
        while (1) {
            delay(1000);
        }
    }

    printHelp();
    delay(1000);

    Serial.println("# Output format: ax:VALUE,ay:VALUE,az:VALUE,amag:VALUE");
    nextSampleMicros = micros();
    lastStatsMillis  = millis();
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    handleSerialCommands();

    if (!sensorAvailable) return;

    // High-precision non-blocking microsecond scheduler
    uint32_t nowMicros = micros();
    if ((int32_t)(nowMicros - nextSampleMicros) >= 0) {
        nextSampleMicros += periodMicros;

        // If loop fell behind by more than 2 periods, resync
        if ((int32_t)(nowMicros - nextSampleMicros) > (int32_t)periodMicros) {
            nextSampleMicros = nowMicros + periodMicros;
        }

        int16_t rx, ry, rz;
        if (readBurstData(sensorAddress, rx, ry, rz)) {
            sampleCountSec++;

            // Convert raw counts to physical acceleration
            const float scale = ADXL_SCALE_G_PER_LSB * (unitIsMps2 ? G_TO_MPS2 : 1.0f);
            float ax = rx * scale - offsetAx;
            float ay = ry * scale - offsetAy;
            float az = rz * scale - offsetAz;
            float amag = sqrtf(ax * ax + ay * ay + az * az);

            // Track peaks for diagnostics
            if (fabsf(ax) > peakAx) peakAx = fabsf(ax);
            if (fabsf(ay) > peakAy) peakAy = fabsf(ay);
            if (fabsf(az) > peakAz) peakAz = fabsf(az);
            if (amag > peakMag)     peakMag = amag;

            // Stream data to Serial Plotter / Monitor
            if (streamingActive) {
                decimationCounter++;
                if (decimationCounter >= decimationFactor) {
                    decimationCounter = 0;
                    // Standard Serial Plotter & Teleplot key:value format
                    Serial.printf("ax:%.3f,ay:%.3f,az:%.3f,amag:%.3f\n",
                                  ax, ay, az, amag);
                }
            }
        }
    }

    // 1-Second Statistics & Actual Frequency Computation
    uint32_t nowMillis = millis();
    if (nowMillis - lastStatsMillis >= 1000UL) {
        uint32_t dt = nowMillis - lastStatsMillis;
        actualRateHz = (sampleCountSec * 1000.0f) / (float)dt;

        // Reset counters
        sampleCountSec   = 0;
        lastStatsMillis  = nowMillis;
        peakAx = peakAy = peakAz = peakMag = 0.0f;
    }
}
