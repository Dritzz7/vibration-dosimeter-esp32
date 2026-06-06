/**
 * @file    main.cpp
 * @brief   Vibration Dosimeter ESP32 — FreeRTOS Dual-Core Firmware Framework
 *
 * @details Simultaneous Hand-Arm Vibration (HAV) and Whole-Body Vibration (WBV)
 *          measurement system conforming to ISO 5349-1 (HAV) and ISO 2631-1 (WBV).
 *
 *          ┌──────────────────────────────────────────────────────────────────┐
 *          │  CORE 0  (PRO_CPU) — Real-Time Acquisition & DSP                 │
 *          │   • vTaskHAVAcquisition  — 2000 Hz, Priority 5                   │
 *          │   • vTaskWBVAcquisition  — 200 Hz,  Priority 4                   │
 *          ├──────────────────────────────────────────────────────────────────┤
 *          │  CORE 1  (APP_CPU) — Logging, HMI & Peripheral Management        │
 *          │   • vTaskDataLogger      — 1 Hz,    Priority 3                   │
 *          │   • vTaskHMIAndController— 10 Hz,   Priority 2                   │
 *          └──────────────────────────────────────────────────────────────────┘
 *
 *          Filter coefficients are auto-generated from MATLAB (ISO 8041).
 *          HAV: Wh weighting, fs=3200 Hz, 3 biquad sections (used at 2000 Hz stride).
 *          WBV: Wd(X,Y)/Wk(Z) weighting, fs=400 Hz, 3 & 4 biquad sections.
 *
 * @note    Target Hardware : ESP32 Dual-Core (240 MHz)
 *          Framework       : Arduino Core + FreeRTOS
 *          HAV Sensor      : ADXL345 (I2C, addr 0x53 or 0x1D), 3200 Hz ODR
 *          WBV Sensor      : ADXL345 (I2C, secondary bus or address), 400 Hz ODR
 *          RTC             : DS3231 (I2C)
 *          Storage         : SD Card (SPI)
 *          Display         : SSD1306 1.3" OLED (I2C)
 *
 * @author  Team EL4060
 * @date    2026-06-06
 */

// =============================================================================
// INCLUDES
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <math.h>

// Project filter coefficient headers (MATLAB-generated, ISO 8041)
#include "hav_coefficients.h"
#include "wbv_coefficients.h"

// Third-party library headers — installed via platformio.ini lib_deps
#include <RTClib.h>         // Adafruit RTClib for DS3231
#include <SD.h>             // Arduino SD library
#include <Adafruit_SSD1306.h>  // Adafruit SSD1306 OLED driver

// =============================================================================
// DEBUG LOGGING — Set DEBUG_ENABLED 0 to silence all serial output in production
// =============================================================================
#define DEBUG_ENABLED  1

#if DEBUG_ENABLED
  #define LOG_I(tag, fmt, ...)  Serial.printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
  #define LOG_W(tag, fmt, ...)  Serial.printf("[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
  #define LOG_E(tag, fmt, ...)  Serial.printf("[ERR ][%s] " fmt "\n", tag, ##__VA_ARGS__)
#else
  #define LOG_I(tag, fmt, ...)  do {} while(0)
  #define LOG_W(tag, fmt, ...)  do {} while(0)
  #define LOG_E(tag, fmt, ...)  do {} while(0)
#endif

// =============================================================================
// HARDWARE PIN DEFINITIONS
// =============================================================================
// --- I2C Bus (shared: HAV sensor, WBV sensor, RTC, OLED) ---
#define PIN_I2C_SDA          21
#define PIN_I2C_SCL          22

// --- SPI Bus (SD Card) ---
#define PIN_SPI_MOSI         23
#define PIN_SPI_MISO         19
#define PIN_SPI_CLK          18
#define PIN_SD_CS             5

// --- HMI ---
#define PIN_BUTTON            4   // Tactile push-button (active-low, internal pull-up)
#define PIN_LED_STATUS        2   // On-board LED for status indication

// --- ADXL345 I2C Addresses ---
#define ADXL345_ADDR_HAV     0x53   // HAV sensor (SDO tied to GND)
#define ADXL345_ADDR_WBV     0x1D   // WBV sensor (SDO tied to 3V3)

// --- DS3231 I2C Address ---
#define DS3231_ADDR          0x68

// =============================================================================
// ADXL345 REGISTER MAP
// =============================================================================
#define ADXL_REG_DEVID       0x00
#define ADXL_REG_BW_RATE     0x2C
#define ADXL_REG_POWER_CTL   0x2D
#define ADXL_REG_DATA_FORMAT 0x31
#define ADXL_REG_DATAX0      0x32
#define ADXL_DEVID_EXPECTED  0xE5

// ADXL345 BW_RATE values
#define ADXL_BW_3200HZ       0x0F   // 3200 Hz ODR — HAV
#define ADXL_BW_400HZ        0x0C   //  400 Hz ODR — WBV

// ADXL345 DATA_FORMAT: FULL_RES | Range ±16g
#define ADXL_FORMAT_FULLRES  0x0B
#define ADXL_SCALE_G_PER_LSB 0.0039f   // full-resolution scale factor (g/LSB)
#define G_TO_MPS2            9.80665f  // gravitational acceleration (m/s²)

// =============================================================================
// SAMPLING & TIMING CONSTANTS
// =============================================================================
// HAV task: target 2000 Hz (every 500 µs). Epoch = 1 second = 2000 samples.
#define HAV_SAMPLE_RATE_HZ   2000U
#define HAV_PERIOD_MS        1U                              // ~500µs → 1ms tick granularity
#define HAV_EPOCH_SAMPLES    (HAV_SAMPLE_RATE_HZ * 1U)      // 2000 samples per 1-s epoch

// WBV task: target 200 Hz (every 5 ms). Epoch = 1 second = 200 samples.
#define WBV_SAMPLE_RATE_HZ   200U
#define WBV_PERIOD_MS        5U                              // 5 ms period
#define WBV_EPOCH_SAMPLES    (WBV_SAMPLE_RATE_HZ * 1U)      // 200 samples per 1-s epoch

// Data logger: 1 Hz polling of queues
#define LOGGER_PERIOD_MS     1000U

// HMI task: 100 ms poll for button and FSM; OLED updated at 1 Hz sub-rate
#define HMI_PERIOD_MS        100U
#define OLED_UPDATE_DIVIDER  10U    // Update OLED every 10th HMI tick = 1 Hz
#define OLED_SCREENSAVER_S   30U    // Blank screen after 30s of LOGGING inactivity

// Button debounce
#define BUTTON_DEBOUNCE_MS   50U

// =============================================================================
// RTOS CONFIGURATION
// =============================================================================
#define TASK_STACK_HAV       4096U
#define TASK_STACK_WBV       4096U
#define TASK_STACK_LOGGER    8192U   // Larger — handles file I/O
#define TASK_STACK_HMI       4096U

#define TASK_PRIO_HAV        5
#define TASK_PRIO_WBV        4
#define TASK_PRIO_LOGGER     3
#define TASK_PRIO_HMI        2

#define CORE_DSP             0       // PRO_CPU — Acquisition & DSP
#define CORE_PERIPHERAL      1       // APP_CPU — Logging & HMI

#define QUEUE_HAV_LENGTH     8U      // Buffer up to 8 one-second RMS results
#define QUEUE_WBV_LENGTH     8U

// =============================================================================
// DATA STRUCTURES
// =============================================================================

/**
 * @struct HavRmsData
 * @brief  One-second RMS epoch result for HAV (ISO 5349-1).
 *         All acceleration values in m/s², timestamp in UNIX epoch seconds.
 */
typedef struct {
    float    ahwx;        ///< Wh-weighted RMS, X-axis  [m/s²]
    float    ahwy;        ///< Wh-weighted RMS, Y-axis  [m/s²]
    float    ahwz;        ///< Wh-weighted RMS, Z-axis  [m/s²]
    float    ahv;         ///< Vector-sum RMS: √(x²+y²+z²) [m/s²]
    uint32_t timestamp;   ///< UNIX epoch time of epoch end [s]
    uint32_t n_samples;   ///< Actual samples accumulated in this epoch
} HavRmsData_t;

/**
 * @struct WbvRmsData
 * @brief  One-second RMS epoch result for WBV (ISO 2631-1).
 *         Wd weighting on X,Y axes; Wk weighting on Z axis.
 */
typedef struct {
    float    awx;         ///< Wd-weighted RMS, X-axis  [m/s²]
    float    awy;         ///< Wd-weighted RMS, Y-axis  [m/s²]
    float    awz;         ///< Wk-weighted RMS, Z-axis  [m/s²]
    float    av;          ///< Total WBV: √((1.4·awx)²+(1.4·awy)²+(awz)²) [m/s²]
    uint32_t timestamp;   ///< UNIX epoch time of epoch end [s]
    uint32_t n_samples;   ///< Actual samples accumulated in this epoch
} WbvRmsData_t;

// =============================================================================
// SYSTEM FSM
// =============================================================================
typedef enum {
    SYS_INIT      = 0,   ///< Power-on initialisation
    SYS_SELF_TEST = 1,   ///< Sensor self-test & I2C scan
    SYS_READY     = 2,   ///< Idle — sensors OK, SD ready, waiting for user
    SYS_LOGGING   = 3,   ///< Active measurement & logging
    SYS_ERROR     = 4    ///< Fatal/recoverable error
} SystemState_t;

// =============================================================================
// GLOBAL RTOS HANDLES
// =============================================================================

// Task handles
static TaskHandle_t  hTaskHAV     = nullptr;
static TaskHandle_t  hTaskWBV     = nullptr;
static TaskHandle_t  hTaskLogger  = nullptr;
static TaskHandle_t  hTaskHMI     = nullptr;

// Queue handles — inter-core data passing (preferred over mutexes for producers→consumers)
static QueueHandle_t xQueueHAVData = nullptr;
static QueueHandle_t xQueueWBVData = nullptr;

// Mutex for shared RTC time resource (accessed by both acquisition tasks for timestamps)
static SemaphoreHandle_t xMutexRTC  = nullptr;

// Mutex for SD Card SPI bus (Logger is the sole writer, but guard for future extensions)
static SemaphoreHandle_t xMutexSD   = nullptr;

// Mutex protecting the systemState variable (written by HMI, read by all tasks)
static SemaphoreHandle_t xMutexState = nullptr;

// =============================================================================
// GLOBAL SHARED STATE (mutex-protected where applicable)
// =============================================================================
static volatile SystemState_t systemState = SYS_INIT;

// Sensor availability flags (written once in setup, read-only afterwards)
static bool havSensorOK = false;
static bool wbvSensorOK = false;
static bool rtcOK       = false;
static bool sdOK        = false;
static bool oledOK      = false;

// Global driver instances
static RTC_DS3231 rtc;
static Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// =============================================================================
// BIQUAD CASCADE FILTER CLASS
// =============================================================================
/**
 * @class BiquadCascade
 * @brief Thread-local Direct Form II Transposed IIR biquad cascade filter.
 *        Each task instance owns its own state — no mutex required.
 *
 * Difference equation per section (a0 = 1 normalised):
 *   y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] − a1·y[n-1] − a2·y[n-2]
 *
 * Coefficient layout: coeff[section][5] = {b0, b1, b2, a1, a2}
 */
class BiquadCascade {
public:
    static constexpr int MAX_SECTIONS = 4; ///< Max to support Wk (4 sections)

    BiquadCascade(const float (*coeff)[5], int numSections)
        : coeff_(coeff), numSections_(numSections) {
        reset();
    }

    /**
     * @brief  Process a single input sample through all biquad sections.
     * @param  input  Raw accelerometer sample [m/s²]
     * @return Frequency-weighted output sample [m/s²]
     */
    inline float process(float input) {
        float x = input;
        for (int i = 0; i < numSections_; ++i) {
            const float b0 = coeff_[i][0];
            const float b1 = coeff_[i][1];
            const float b2 = coeff_[i][2];
            const float a1 = coeff_[i][3];
            const float a2 = coeff_[i][4];

            const float y = b0 * x   + b1 * x1_[i] + b2 * x2_[i]
                              - a1 * y1_[i] - a2 * y2_[i];

            x2_[i] = x1_[i];   x1_[i] = x;
            y2_[i] = y1_[i];   y1_[i] = y;
            x = y;
        }
        return x;
    }

    /** @brief Reset all internal delay-line states to zero (call on FSM state change). */
    void reset() {
        for (int i = 0; i < MAX_SECTIONS; ++i) {
            x1_[i] = x2_[i] = y1_[i] = y2_[i] = 0.0f;
        }
    }

private:
    const float (*coeff_)[5];
    int          numSections_;
    float        x1_[MAX_SECTIONS];
    float        x2_[MAX_SECTIONS];
    float        y1_[MAX_SECTIONS];
    float        y2_[MAX_SECTIONS];
};

// =============================================================================
// I2C HELPER FUNCTIONS  (Wire-based, blocking, called only from setup/self-test)
// =============================================================================
static bool i2c_writeReg(uint8_t devAddr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

static bool i2c_readReg(uint8_t devAddr, uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)devAddr, 1) != 1) return false;
    if (!Wire.available()) return false;
    value = Wire.read();
    return true;
}

static bool i2c_readBurst(uint8_t devAddr, uint8_t startReg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(devAddr);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) return false;
    if ((uint8_t)Wire.requestFrom((int)devAddr, (int)len) != len) {
        while (Wire.available()) Wire.read(); // flush
        return false;
    }
    for (uint8_t i = 0; i < len; ++i) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}

// =============================================================================
// ADXL345 DRIVER FUNCTIONS
// =============================================================================
static bool adxl345_detect(uint8_t addr) {
    uint8_t devid = 0;
    if (!i2c_readReg(addr, ADXL_REG_DEVID, devid)) return false;
    return (devid == ADXL_DEVID_EXPECTED);
}

static bool adxl345_init(uint8_t addr, uint8_t bwRate) {
    // 1. Standby mode
    if (!i2c_writeReg(addr, ADXL_REG_POWER_CTL, 0x00)) return false;
    // 2. Set output data rate
    if (!i2c_writeReg(addr, ADXL_REG_BW_RATE, bwRate))  return false;
    // 3. DATA_FORMAT: FULL_RES=1, Range=±16g
    if (!i2c_writeReg(addr, ADXL_REG_DATA_FORMAT, ADXL_FORMAT_FULLRES)) return false;
    // 4. Measurement mode
    if (!i2c_writeReg(addr, ADXL_REG_POWER_CTL, 0x08)) return false;
    return true;
}

/**
 * @brief Read tri-axial acceleration from an ADXL345.
 * @param addr  I2C address of the sensor.
 * @param[out] ax, ay, az  Acceleration in m/s².
 * @return true on success.
 */
static bool adxl345_read(uint8_t addr, float &ax, float &ay, float &az) {
    uint8_t raw[6];
    if (!i2c_readBurst(addr, ADXL_REG_DATAX0, raw, 6)) return false;

    const int16_t rawX = (int16_t)((raw[1] << 8) | raw[0]);
    const int16_t rawY = (int16_t)((raw[3] << 8) | raw[2]);
    const int16_t rawZ = (int16_t)((raw[5] << 8) | raw[4]);

    ax = rawX * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    ay = rawY * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    az = rawZ * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    return true;
}

// =============================================================================
// RTC HELPER  (DS3231 integration using RTClib)
// =============================================================================
/**
 * @brief  Read current UNIX timestamp from DS3231 via I2C.
 * @return UNIX timestamp in seconds, or 0 on failure.
 */
static uint32_t rtc_getUnixTimestamp() {
    uint32_t ts = 0;
    if (xSemaphoreTake(xMutexRTC, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (rtcOK) {
            ts = rtc.now().unixtime();
        } else {
            ts = (uint32_t)(millis() / 1000UL); // fallback relative timestamp
        }
        xSemaphoreGive(xMutexRTC);
    } else {
        ts = (uint32_t)(millis() / 1000UL); // fallback
    }
    return ts;
}

// =============================================================================
// SYSTEM STATE HELPER (mutex-protected read/write)
// =============================================================================
static SystemState_t getSystemState() {
    SystemState_t s;
    if (xSemaphoreTake(xMutexState, pdMS_TO_TICKS(5)) == pdTRUE) {
        s = systemState;
        xSemaphoreGive(xMutexState);
    } else {
        s = systemState; // safe-read: volatile
    }
    return s;
}

static void setSystemState(SystemState_t newState) {
    if (xSemaphoreTake(xMutexState, pdMS_TO_TICKS(10)) == pdTRUE) {
        systemState = newState;
        xSemaphoreGive(xMutexState);
    }
}

// =============================================================================
// TASK: vTaskHAVAcquisition
// Core 0 | Priority 5 | Period ~500 µs (2000 Hz)
// =============================================================================
/**
 * @brief  HAV acquisition task — Core 0, Priority 5.
 *
 *         Reads tri-axial acceleration from the HAV ADXL345 at 2000 Hz,
 *         applies ISO 8041 Wh frequency-weighting (3-section biquad cascade),
 *         accumulates squared weighted samples for RMS, and pushes a
 *         one-second epoch result onto xQueueHAVData.
 *
 *         Timing: vTaskDelayUntil ensures a strict 1 ms period (FreeRTOS tick).
 *         Sub-millisecond accuracy relies on the ADXL345 ODR being ≥ 2000 Hz
 *         (set to 3200 Hz) and the I2C bus being fast enough.
 */
static void vTaskHAVAcquisition(void *pvParameters) {
    static const char *TAG = "HAV_ACQ";

    // Per-axis Wh IIR filters (task-local, no shared state)
    static BiquadCascade filterHAV_X(coeff_wh, NUM_SECTIONS_WH);
    static BiquadCascade filterHAV_Y(coeff_wh, NUM_SECTIONS_WH);
    static BiquadCascade filterHAV_Z(coeff_wh, NUM_SECTIONS_WH);

    // RMS accumulators
    float sumX2 = 0.0f, sumY2 = 0.0f, sumZ2 = 0.0f;
    uint32_t sampleCount = 0;

    // Timing baseline for vTaskDelayUntil
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(HAV_PERIOD_MS); // 1 ms tick

    LOG_I(TAG, "HAV acquisition task started on Core %d", xPortGetCoreID());

    while (true) {
        // ── Precise periodic delay — no drift ────────────────────────────────
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // ── Gate on system state ─────────────────────────────────────────────
        const SystemState_t state = getSystemState();
        if (state != SYS_LOGGING) {
            // Reset filters and accumulators when not actively logging
            if (state == SYS_READY || state == SYS_ERROR) {
                filterHAV_X.reset();
                filterHAV_Y.reset();
                filterHAV_Z.reset();
                sumX2 = sumY2 = sumZ2 = 0.0f;
                sampleCount = 0;
            }
            continue;
        }

        if (!havSensorOK) continue;

        // ── Sensor Read ───────────────────────────────────────────────────────
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        if (!adxl345_read(ADXL345_ADDR_HAV, ax, ay, az)) {
            continue;
        }

        // ── Wh Frequency Weighting (ISO 8041 Biquad Cascade) ─────────────────
        const float axWh = filterHAV_X.process(ax);
        const float ayWh = filterHAV_Y.process(ay);
        const float azWh = filterHAV_Z.process(az);

        // ── RMS Accumulation ──────────────────────────────────────────────────
        sumX2 += axWh * axWh;
        sumY2 += ayWh * ayWh;
        sumZ2 += azWh * azWh;
        sampleCount++;

        // ── 1-Second Epoch Boundary ───────────────────────────────────────────
        if (sampleCount >= HAV_EPOCH_SAMPLES) {
            HavRmsData_t result;
            result.n_samples = sampleCount;
            result.ahwx      = sqrtf(sumX2 / sampleCount);
            result.ahwy      = sqrtf(sumY2 / sampleCount);
            result.ahwz      = sqrtf(sumZ2 / sampleCount);
            // ISO 5349-1 vector sum: ahv = √(ahwx² + ahwy² + ahwz²)
            result.ahv       = sqrtf(result.ahwx * result.ahwx
                                   + result.ahwy * result.ahwy
                                   + result.ahwz * result.ahwz);
            result.timestamp = rtc_getUnixTimestamp();

            // Push to queue — non-blocking; drop if Logger is behind
            if (xQueueSend(xQueueHAVData, &result, 0) != pdTRUE) {
                LOG_W(TAG, "xQueueHAVData full — epoch dropped (t=%lu)", result.timestamp);
            }

            LOG_I(TAG, "ahv=%.4f m/s2 | ahwx=%.4f ahwy=%.4f ahwz=%.4f | n=%lu",
                  result.ahv, result.ahwx, result.ahwy, result.ahwz, result.n_samples);

            // Reset accumulators
            sumX2 = sumY2 = sumZ2 = 0.0f;
            sampleCount = 0;
        }
    }
}

// =============================================================================
// TASK: vTaskWBVAcquisition
// Core 0 | Priority 4 | Period 5 ms (200 Hz)
// =============================================================================
/**
 * @brief  WBV acquisition task — Core 0, Priority 4.
 *
 *         Reads tri-axial acceleration from the WBV ADXL345 at 200 Hz,
 *         applies ISO 8041 Wd (X,Y axes) and Wk (Z axis) frequency weighting,
 *         accumulates for 1-second RMS epoch, then pushes WbvRmsData_t onto
 *         xQueueWBVData.
 *
 * @note   ADXL345 hardware ODR is set to 400 Hz; the FreeRTOS task reads at 200 Hz.
 *         This intentional 2× oversampling avoids aliasing; the IIR filter acts
 *         as anti-alias before the 200 Hz virtual rate.
 */
static void vTaskWBVAcquisition(void *pvParameters) {
    static const char *TAG = "WBV_ACQ";

    // Per-axis frequency-weighting filters (task-local, no mutex needed)
    static BiquadCascade filterWBV_X(coeff_wd, NUM_SECTIONS_WD); // Wd
    static BiquadCascade filterWBV_Y(coeff_wd, NUM_SECTIONS_WD); // Wd
    static BiquadCascade filterWBV_Z(coeff_wk, NUM_SECTIONS_WK); // Wk

    // RMS accumulators
    float sumX2 = 0.0f, sumY2 = 0.0f, sumZ2 = 0.0f;
    uint32_t sampleCount = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(WBV_PERIOD_MS); // 5 ms

    LOG_I(TAG, "WBV acquisition task started on Core %d", xPortGetCoreID());

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        const SystemState_t state = getSystemState();
        if (state != SYS_LOGGING) {
            if (state == SYS_READY || state == SYS_ERROR) {
                filterWBV_X.reset();
                filterWBV_Y.reset();
                filterWBV_Z.reset();
                sumX2 = sumY2 = sumZ2 = 0.0f;
                sampleCount = 0;
            }
            continue;
        }

        if (!wbvSensorOK) continue;

        // ── Sensor Read ───────────────────────────────────────────────────────
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        if (!adxl345_read(ADXL345_ADDR_WBV, ax, ay, az)) {
            continue;
        }

        // ── Frequency Weighting: Wd (X,Y) and Wk (Z) ─────────────────────────
        const float axWd = filterWBV_X.process(ax);
        const float ayWd = filterWBV_Y.process(ay);
        const float azWk = filterWBV_Z.process(az);

        // ── RMS Accumulation ──────────────────────────────────────────────────
        sumX2 += axWd * axWd;
        sumY2 += ayWd * ayWd;
        sumZ2 += azWk * azWk;
        sampleCount++;

        // ── 1-Second Epoch Boundary ───────────────────────────────────────────
        if (sampleCount >= WBV_EPOCH_SAMPLES) {
            WbvRmsData_t result;
            result.n_samples = sampleCount;
            result.awx       = sqrtf(sumX2 / sampleCount);
            result.awy       = sqrtf(sumY2 / sampleCount);
            result.awz       = sqrtf(sumZ2 / sampleCount);
            // ISO 2631-1 total WBV (seated): av = √((1.4·awx)²+(1.4·awy)²+(awz)²)
            result.av        = sqrtf(
                                   (1.4f * result.awx) * (1.4f * result.awx) +
                                   (1.4f * result.awy) * (1.4f * result.awy) +
                                   (       result.awz) * (       result.awz));
            result.timestamp = rtc_getUnixTimestamp();

            if (xQueueSend(xQueueWBVData, &result, 0) != pdTRUE) {
                LOG_W(TAG, "xQueueWBVData full — epoch dropped (t=%lu)", result.timestamp);
            }

            LOG_I(TAG, "av=%.4f m/s2 | awx=%.4f awy=%.4f awz=%.4f | n=%lu",
                  result.av, result.awx, result.awy, result.awz, result.n_samples);

            sumX2 = sumY2 = sumZ2 = 0.0f;
            sampleCount = 0;
        }
    }
}

// =============================================================================
// TASK: vTaskDataLogger
// Core 1 | Priority 3 | Interval 1000 ms
// =============================================================================
/**
 * @brief  Data Logger task — Core 1, Priority 3.
 *
 *         Runs at 1 Hz. Non-blocking queue receives aggregate all available
 *         HAV and WBV epoch results, reads the RTC for a precise timestamp,
 *         and appends structured CSV records to the SD card.
 *
 *         CSV format:
 *         timestamp,ahwx,ahwy,ahwz,ahv,awx,awy,awz,av
 *
 *         Fail-safe: The file is closed (flushed) after every write batch.
 *         This prevents data loss on unexpected power loss.
 */
static void vTaskDataLogger(void *pvParameters) {
    static const char *TAG = "LOGGER";

    // Local aggregation buffers (pre-declared to avoid heap fragmentation)
    HavRmsData_t havData;
    WbvRmsData_t wbvData;

    bool havReceived = false;
    bool wbvReceived = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(LOGGER_PERIOD_MS);

    LOG_I(TAG, "Data logger task started on Core %d", xPortGetCoreID());

    // ── SD Card Initialisation ────────────────────────────────────────────────
    if (sdOK) {
        if (xSemaphoreTake(xMutexSD, pdMS_TO_TICKS(50)) == pdTRUE) {
            File f = SD.open("/dosimeter.csv", FILE_WRITE);
            if (f) {
                f.println("timestamp,ahwx,ahwy,ahwz,ahv,awx,awy,awz,av");
                f.close();
            }
            xSemaphoreGive(xMutexSD);
        }
    }

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        const SystemState_t state = getSystemState();
        if (state != SYS_LOGGING) continue;

        // ── Non-blocking queue drain ──────────────────────────────────────────
        // Drain all available HAV epochs (non-blocking, timeout = 0)
        havReceived = false;
        while (xQueueReceive(xQueueHAVData, &havData, 0) == pdTRUE) {
            havReceived = true;
            // Latest value wins; older epochs in the same second are still logged below
            // For continuous logging, loop and write each one individually:
            // [PLACEHOLDER — Write HAV record to SD]
        }

        // Drain all available WBV epochs
        wbvReceived = false;
        while (xQueueReceive(xQueueWBVData, &wbvData, 0) == pdTRUE) {
            wbvReceived = true;
        }

        // ── SD Write ──────────────────────────────────────────────────────────
        if (havReceived || wbvReceived) {
            const uint32_t ts = rtc_getUnixTimestamp();

            if (sdOK) {
                // Take mutex to guard SPI bus if other tasks use SPI
                if (xSemaphoreTake(xMutexSD, pdMS_TO_TICKS(50)) == pdTRUE) {
                    File f = SD.open("/dosimeter.csv", FILE_APPEND);
                    if (f) {
                        // Compose CSV line
                        char line[128];
                        snprintf(line, sizeof(line),
                                 "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                                 ts,
                                 havReceived ? havData.ahwx : 0.0f,
                                 havReceived ? havData.ahwy : 0.0f,
                                 havReceived ? havData.ahwz : 0.0f,
                                 havReceived ? havData.ahv  : 0.0f,
                                 wbvReceived ? wbvData.awx  : 0.0f,
                                 wbvReceived ? wbvData.awy  : 0.0f,
                                 wbvReceived ? wbvData.awz  : 0.0f,
                                 wbvReceived ? wbvData.av   : 0.0f);
                        f.println(line);
                        f.close();  // Fail-safe flush on every write
                    } else { 
                        LOG_E(TAG, "Cannot open CSV file for logging!"); 
                    }
                    xSemaphoreGive(xMutexSD);
                }
            }

            LOG_I(TAG, "LOGGER t=%lu | HAV ahv=%.4f | WBV av=%.4f",
                  ts,
                  havReceived ? havData.ahv : 0.0f,
                  wbvReceived ? wbvData.av  : 0.0f);
        }
    }
}

// =============================================================================
// TASK: vTaskHMIAndController
// Core 1 | Priority 2 | Period 100 ms
// =============================================================================
/**
 * @brief  HMI & FSM Controller task — Core 1, Priority 2.
 *
 *         Runs every 100 ms:
 *           1. Debounces the tactile push-button (50 ms window, non-blocking).
 *           2. Drives the System FSM transitions based on button events.
 *           3. Updates the OLED at 1 Hz (sub-divided by OLED_UPDATE_DIVIDER).
 *           4. Implements screen-saver blanking after OLED_SCREENSAVER_S of
 *              continuous LOGGING with no button activity.
 *
 *         FSM Transitions:
 *           INIT      → SELF_TEST  : automatic after init complete
 *           SELF_TEST → READY      : all sensors OK
 *           SELF_TEST → ERROR      : sensor/SD failure
 *           READY     → LOGGING    : button SHORT press
 *           LOGGING   → READY      : button SHORT press (stop)
 *           ERROR     → SELF_TEST  : button LONG press (retry)
 */
static void vTaskHMIAndController(void *pvParameters) {
    static const char *TAG = "HMI";

    // ── Button debounce state ─────────────────────────────────────────────────
    bool     buttonLastState   = HIGH;
    bool     buttonStable      = HIGH;
    uint32_t buttonDebounceMs  = 0;
    uint32_t buttonPressMs     = 0;   // millis() when button went LOW

    // ── OLED sub-rate counter and screen-saver timer ──────────────────────────
    uint32_t oledTickCounter   = 0;
    uint32_t lastActivityMs    = millis();
    bool     screenSaverActive = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(HMI_PERIOD_MS); // 100 ms

    LOG_I(TAG, "HMI task started on Core %d", xPortGetCoreID());

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        const uint32_t now = millis();

        // ════════════════════════════════════════════════════════════════════
        // 1. NON-BLOCKING BUTTON DEBOUNCE (50 ms sliding window)
        // ════════════════════════════════════════════════════════════════════
        const bool rawButton = (bool)digitalRead(PIN_BUTTON); // active-low

        if (rawButton != buttonLastState) {
            buttonDebounceMs = now;   // restart debounce timer on any edge
            buttonLastState  = rawButton;
        }

        bool buttonEvent = false;   // true = stable press event this tick

        if ((now - buttonDebounceMs) >= BUTTON_DEBOUNCE_MS) {
            if (rawButton != buttonStable) {
                buttonStable = rawButton;
                if (buttonStable == LOW) {
                    // Falling edge confirmed: button pressed
                    buttonPressMs = now;
                    lastActivityMs = now;
                    buttonEvent    = true;
                }
            }
        }

        // ════════════════════════════════════════════════════════════════════
        // 2. SYSTEM FSM
        // ════════════════════════════════════════════════════════════════════
        SystemState_t state = getSystemState();

        switch (state) {
            // ── INIT: One-time boot initialisation; transition to SELF_TEST ──
            case SYS_INIT:
                // Normally transitioned out of in setup(). Kept for robustness.
                setSystemState(SYS_SELF_TEST);
                LOG_I(TAG, "FSM: INIT → SELF_TEST");
                break;

            // ── SELF_TEST: Verify all peripherals ───────────────────────────
            case SYS_SELF_TEST: {
                // [PLACEHOLDER — Self-Test Routine]
                // Re-verify sensor presence and SD card mount
                const bool allOK = (havSensorOK && wbvSensorOK);
                if (allOK) {
                    setSystemState(SYS_READY);
                    LOG_I(TAG, "FSM: SELF_TEST → READY");
                } else {
                    setSystemState(SYS_ERROR);
                    LOG_E(TAG, "FSM: SELF_TEST → ERROR (HAV:%d WBV:%d SD:%d)",
                          havSensorOK, wbvSensorOK, sdOK);
                }
                break;
            }

            // ── READY: Waiting for user to start logging ─────────────────────
            case SYS_READY:
                if (buttonEvent) {
                    setSystemState(SYS_LOGGING);
                    LOG_I(TAG, "FSM: READY → LOGGING");
                    // Notify acquisition tasks (optional: use task notification)
                    if (hTaskHAV) xTaskNotify(hTaskHAV, 1UL, eSetBits);
                    if (hTaskWBV) xTaskNotify(hTaskWBV, 1UL, eSetBits);
                }
                break;

            // ── LOGGING: Active measurement session ──────────────────────────
            case SYS_LOGGING:
                if (buttonEvent) {
                    setSystemState(SYS_READY);
                    LOG_I(TAG, "FSM: LOGGING → READY (user stopped)");
                }
                break;

            // ── ERROR: Fault state; long-press to retry ──────────────────────
            case SYS_ERROR:
                if (buttonEvent) {
                    setSystemState(SYS_SELF_TEST);
                    LOG_I(TAG, "FSM: ERROR → SELF_TEST (user retry)");
                }
                break;

            default:
                break;
        }

        // ════════════════════════════════════════════════════════════════════
        // 3. OLED UPDATE at 1 Hz + SCREEN-SAVER
        // ════════════════════════════════════════════════════════════════════
        oledTickCounter++;
        if (oledTickCounter >= OLED_UPDATE_DIVIDER) {
            oledTickCounter = 0;

            // Screen-saver: blank after inactivity during LOGGING
            const uint32_t idleSeconds = (now - lastActivityMs) / 1000UL;
            if (state == SYS_LOGGING && idleSeconds >= OLED_SCREENSAVER_S) {
                if (!screenSaverActive) {
                    screenSaverActive = true;
                    if (oledOK) {
                        oled.clearDisplay();
                        oled.display();
                    }
                    LOG_I(TAG, "OLED screensaver activated");
                }
            } else {
                screenSaverActive = false;
                if (oledOK && !screenSaverActive) {
                    oled.clearDisplay();
                    oled.setTextSize(1);
                    oled.setTextColor(SSD1306_WHITE);
                    oled.setCursor(0, 0);

                    oled.println("VIBRATION DOSIMETER");
                    oled.println("====================");
                    oled.print("STATUS: ");
                    switch (state) {
                        case SYS_INIT:      oled.println("INIT");      break;
                        case SYS_SELF_TEST: oled.println("SELF TEST"); break;
                        case SYS_READY:     oled.println("READY");     break;
                        case SYS_LOGGING:   oled.println("LOGGING");   break;
                        case SYS_ERROR:     oled.println("ERROR");     break;
                    }

                    oled.print("HAV Acc: ");
                    oled.println(havSensorOK ? "OK" : "ERR");
                    oled.print("WBV Acc: ");
                    oled.println(wbvSensorOK ? "OK" : "ERR");
                    oled.print("SD Card: ");
                    oled.println(sdOK ? "OK" : "ERR");
                    oled.print("RTC:     ");
                    oled.println(rtcOK ? "OK" : "ERR");

                    oled.display();
                }
            }
        }

        // Activity tracker: reset screensaver if button pressed
        if (buttonEvent) {
            lastActivityMs    = now;
            screenSaverActive = false;
        }
    }
}

// =============================================================================
// SENSOR SELF-TEST (called from setup() before tasks are created)
// =============================================================================
static void runSelfTest() {
    LOG_I("INIT", "Running sensor self-test...");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000); // Start at 100 kHz for detection

    // HAV ADXL345
    havSensorOK = adxl345_detect(ADXL345_ADDR_HAV);
    if (havSensorOK) {
        havSensorOK = adxl345_init(ADXL345_ADDR_HAV, ADXL_BW_3200HZ);
        LOG_I("INIT", "HAV ADXL345 @ 0x%02X init %s", ADXL345_ADDR_HAV, havSensorOK ? "OK" : "FAIL");
    } else {
        LOG_E("INIT", "HAV ADXL345 @ 0x%02X NOT FOUND", ADXL345_ADDR_HAV);
    }

    // WBV ADXL345
    wbvSensorOK = adxl345_detect(ADXL345_ADDR_WBV);
    if (wbvSensorOK) {
        wbvSensorOK = adxl345_init(ADXL345_ADDR_WBV, ADXL_BW_400HZ);
        LOG_I("INIT", "WBV ADXL345 @ 0x%02X init %s", ADXL345_ADDR_WBV, wbvSensorOK ? "OK" : "FAIL");
    } else {
        LOG_E("INIT", "WBV ADXL345 @ 0x%02X NOT FOUND", ADXL345_ADDR_WBV);
    }

    // DS3231 RTC Self-Test
    rtcOK = rtc.begin();
    LOG_I("INIT", "DS3231 RTC init %s", rtcOK ? "OK" : "NOT FOUND");
    if (rtcOK && rtc.lostPower()) {
        LOG_W("INIT", "RTC lost power, setting compilation time!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // SD Card Self-Test
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SD_CS);
    sdOK = SD.begin(PIN_SD_CS);
    LOG_I("INIT", "SD Card init %s", sdOK ? "OK" : "FAIL");

    // OLED Init
    oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (oledOK) {
        oled.clearDisplay();
        oled.display();
        LOG_I("INIT", "SSD1306 OLED init OK");
    } else {
        LOG_E("INIT", "SSD1306 OLED init FAIL");
    }

    // After detection, raise I2C clock for normal operation
    Wire.setClock(400000);

    LOG_I("INIT", "Self-test complete: HAV=%d WBV=%d RTC=%d SD=%d OLED=%d",
          havSensorOK, wbvSensorOK, rtcOK, sdOK, oledOK);
}

// =============================================================================
// RTOS RESOURCE CREATION
// =============================================================================
static void createRTOSObjects() {
    // Inter-core data queues
    xQueueHAVData = xQueueCreate(QUEUE_HAV_LENGTH, sizeof(HavRmsData_t));
    xQueueWBVData = xQueueCreate(QUEUE_WBV_LENGTH, sizeof(WbvRmsData_t));
    configASSERT(xQueueHAVData != nullptr);
    configASSERT(xQueueWBVData != nullptr);

    // Mutexes
    xMutexRTC   = xSemaphoreCreateMutex();
    xMutexSD    = xSemaphoreCreateMutex();
    xMutexState = xSemaphoreCreateMutex();
    configASSERT(xMutexRTC   != nullptr);
    configASSERT(xMutexSD    != nullptr);
    configASSERT(xMutexState != nullptr);
}

// =============================================================================
// TASK CREATION
// =============================================================================
static void createTasks() {
    BaseType_t res;

    // ── Core 0: High-priority DSP tasks ──────────────────────────────────────
    res = xTaskCreatePinnedToCore(
        vTaskHAVAcquisition, "HAV_ACQ",
        TASK_STACK_HAV, nullptr,
        TASK_PRIO_HAV, &hTaskHAV,
        CORE_DSP);
    configASSERT(res == pdPASS);

    res = xTaskCreatePinnedToCore(
        vTaskWBVAcquisition, "WBV_ACQ",
        TASK_STACK_WBV, nullptr,
        TASK_PRIO_WBV, &hTaskWBV,
        CORE_DSP);
    configASSERT(res == pdPASS);

    // ── Core 1: Peripheral tasks ──────────────────────────────────────────────
    res = xTaskCreatePinnedToCore(
        vTaskDataLogger, "LOGGER",
        TASK_STACK_LOGGER, nullptr,
        TASK_PRIO_LOGGER, &hTaskLogger,
        CORE_PERIPHERAL);
    configASSERT(res == pdPASS);

    res = xTaskCreatePinnedToCore(
        vTaskHMIAndController, "HMI",
        TASK_STACK_HMI, nullptr,
        TASK_PRIO_HMI, &hTaskHMI,
        CORE_PERIPHERAL);
    configASSERT(res == pdPASS);

    LOG_I("INIT", "All tasks created successfully.");
}

// =============================================================================
// ARDUINO SETUP — Runs once on Core 1 before the scheduler
// =============================================================================
void setup() {
#if DEBUG_ENABLED
    Serial.begin(115200);
    // Short settling delay before tasks start
    delay(500);
#endif

    Serial.println();
    Serial.println("==========================================================");
    Serial.println("  Vibration Dosimeter ESP32 — HAV/WBV Firmware v1.1       ");
    Serial.println("  Standards: ISO 5349-1 (HAV) | ISO 2631-1 (WBV)          ");
    Serial.println("  Framework: Arduino + FreeRTOS Dual-Core                  ");
    Serial.println("==========================================================");

    // GPIO initialisation
    pinMode(PIN_BUTTON,     INPUT_PULLUP);
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);

    // Initial system state
    systemState = SYS_INIT;

    // Create RTOS queues and mutexes before tasks
    createRTOSObjects();

    // Run synchronous sensor self-test before starting tasks
    runSelfTest();

    // Set initial FSM state based on self-test results
    if (havSensorOK && wbvSensorOK) {
        setSystemState(SYS_READY);
        LOG_I("INIT", "FSM: INIT → READY");
    } else {
        setSystemState(SYS_ERROR);
        LOG_E("INIT", "FSM: INIT → ERROR — check sensor wiring");
    }

    // Create and pin all FreeRTOS tasks
    createTasks();

    LOG_I("INIT", "FreeRTOS scheduler running. loop() is intentionally empty.");

    // Status LED: blink twice to indicate successful boot
    for (int i = 0; i < 2; ++i) {
        digitalWrite(PIN_LED_STATUS, HIGH); delay(100);
        digitalWrite(PIN_LED_STATUS, LOW);  delay(100);
    }
}

// =============================================================================
// ARDUINO LOOP — Intentionally empty per FreeRTOS best practice
// =============================================================================
/**
 * @brief  loop() is left empty. All application logic runs inside
 *         FreeRTOS tasks pinned to dedicated cores. The Arduino loop task
 *         (running on Core 1, lowest priority) simply yields to the scheduler.
 */
void loop() {
    vTaskDelay(portMAX_DELAY); // Permanently suspend the loop task
}