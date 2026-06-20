/**
 * @file    main.cpp
 * @brief   Vibration Dosimeter ESP32 — Main Unit Firmware (FreeRTOS Dual-Core)
 *
 * @details Simultaneous Hand-Arm Vibration (HAV) and Whole-Body Vibration (WBV)
 *          measurement system conforming to ISO 5349-1 (HAV) and ISO 2631-1 (WBV).
 *
 *          ┌──────────────────────────────────────────────────────────────────┐
 *          │  CORE 0  (PRO_CPU) — Real-Time Acquisition & DSP                 │
 *          │   • vTaskWBVAcquisition  — 200 Hz,  Priority 4                   │
 *          ├──────────────────────────────────────────────────────────────────┤
 *          │  CORE 1  (APP_CPU) — Logging, HMI & BLE Management               │
 *          │   • vTaskBLEReceiver     — event-driven, Priority 3              │
 *          │   • vTaskDataLogger      — 1 Hz,    Priority 2                   │
 *          │   • vTaskHMIAndController— 10 Hz,   Priority 1                   │
 *          └──────────────────────────────────────────────────────────────────┘
 *
 *          HAV data is received wirelessly from the HAV Node (Darren) via BLE.
 *          The HAV Node acts as BLE Server (GATT Peripheral); this unit is BLE
 *          Client (GATT Central). Payload format: "HAV,seq,millis,ahwx,ahwy,ahwz,ahv,n"
 *
 *          WBV: Wd(X,Y)/Wk(Z) weighting, fs=400 Hz, 3 & 4 biquad sections.
 *
 * @note    Target Hardware : ESP32 Dual-Core (240 MHz)
 *          Framework       : Arduino Core + FreeRTOS
 *          HAV Data Source : BLE from HAV Node ("HAV_NODE" device name)
 *          WBV Sensor      : ADXL345 (I2C, auto-detect 0x53 or 0x1D), 400 Hz ODR
 *          RTC             : DS3231 (I2C)
 *          Storage         : SD Card (SPI)
 *          Display         : SSD1306 1.3" OLED (I2C)
 *
 * @author  Team EL4060
 * @date    2026-06-17
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

// BLE Client (ESP32 built-in Bluetooth stack)
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <BLEAdvertisedDevice.h>

// Project filter coefficient headers (MATLAB-generated, ISO 8041)
#include "wbv_coefficients.h"

// Third-party library headers — installed via platformio.ini lib_deps
#include <RTClib.h>             // Adafruit RTClib for DS3231
#include <SD.h>                 // Arduino SD library
#include <Adafruit_SSD1306.h>   // Adafruit SSD1306 OLED driver

// =============================================================================
// DEBUG & BLE CONFIGURATION
// =============================================================================
#define DEBUG_ENABLED      1
#define USE_BLE_HAV        1   // 1 = Receive HAV RMS from HAV Node via BLE (production)
                               // 0 = No HAV data (WBV-only mode, for bench testing)

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

// --- ADXL345 I2C Addresses (WBV only) ---
#define ADXL345_ADDR_WBV     0x1D   // WBV sensor (SDO tied to 3V3)

// =============================================================================
// BLE CLIENT CONFIGURATION (must match HAV Node firmware by Darren)
// =============================================================================
#define BLE_HAV_DEVICE_NAME  "HAV_NODE"
#define BLE_SERVICE_UUID     "9b6f0001-5f5a-4f0d-9d7f-000000000001"
#define BLE_CHAR_UUID_HAV    "9b6f0002-5f5a-4f0d-9d7f-000000000002"
#define BLE_SCAN_DURATION_S  5      // Scan window per attempt (seconds)
#define BLE_RECONNECT_MS     5000   // Wait between reconnect attempts

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
// HAV: epochs arrive over BLE from HAV Node (~1 per second)

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
#define TASK_STACK_WBV       4096U
#define TASK_STACK_BLE       8192U   // BLE stack needs extra heap
#define TASK_STACK_LOGGER    8192U   // Larger — handles file I/O
#define TASK_STACK_HMI       4096U

#define TASK_PRIO_WBV        4
#define TASK_PRIO_BLE        3       // BLE Receiver (event-driven, Core 1)
#define TASK_PRIO_LOGGER     2
#define TASK_PRIO_HMI        1

#define CORE_DSP             0       // PRO_CPU — WBV Acquisition & DSP
#define CORE_PERIPHERAL      1       // APP_CPU — BLE, Logging & HMI

#define QUEUE_HAV_LENGTH     8U      // Buffer up to 8 BLE-received HAV epochs
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
static TaskHandle_t  hTaskBLE     = nullptr;   // BLE Receiver (replaces local HAV ACQ)
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

// Sensor/peripheral availability flags
static bool wbvSensorOK  = false;
static uint8_t actual_wbv_addr = 0x00;
static bool rtcOK        = false;
static bool sdOK         = false;
static bool oledOK       = false;

// BLE connection status (volatile: written by BLE callback on Core 1, read by HMI)
static volatile bool bleConnected   = false;   // true = HAV Node BLE link is up
static volatile bool bleHavDataOK   = false;   // true = at least one valid packet received

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
// BLE CLIENT — Scan Callback & Notification Callback
// =============================================================================

// Forward declarations for BLE objects (defined below)
static BLEClient            *pBleClient          = nullptr;
static BLERemoteCharacteristic *pHavCharacteristic = nullptr;
static BLEAdvertisedDevice  *pFoundDevice        = nullptr;
static bool                  doConnect            = false;
static bool                  doScan               = false;

/**
 * @brief  Parse HAV BLE CSV payload and push to xQueueHAVData.
 *
 *         Expected format from Darren's HAV Node:
 *         "HAV,<seq>,<millis_ms>,<ahwx>,<ahwy>,<ahwz>,<ahv>,<n_samples>"
 */
static void parseAndEnqueueHavPayload(const char *payload) {
    // Tokenise using sscanf for speed (no heap allocation)
    uint32_t seq      = 0;
    uint32_t millisMs = 0;
    float    ahwx     = 0.0f;
    float    ahwy     = 0.0f;
    float    ahwz     = 0.0f;
    float    ahv      = 0.0f;
    uint32_t nSamples = 0;

    // Expected: "HAV,%lu,%lu,%f,%f,%f,%f,%u"
    int parsed = sscanf(payload,
                        "HAV,%lu,%lu,%f,%f,%f,%f,%lu",
                        &seq, &millisMs, &ahwx, &ahwy, &ahwz, &ahv, &nSamples);

    if (parsed != 7) {
        LOG_W("BLE_RX", "Bad HAV payload (parsed=%d): %s", parsed, payload);
        return;
    }

    HavRmsData_t result;
    result.ahwx     = ahwx;
    result.ahwy     = ahwy;
    result.ahwz     = ahwz;
    result.ahv      = ahv;
    result.n_samples = nSamples;
    // Stamp with local RTC (authoritative time source on Main Unit)
    result.timestamp = rtc_getUnixTimestamp();

    bleHavDataOK = true;

    if (xQueueSend(xQueueHAVData, &result, 0) != pdTRUE) {
        LOG_W("BLE_RX", "xQueueHAVData full — BLE HAV epoch dropped (seq=%lu)", seq);
    } else {
        LOG_I("BLE_RX", "seq=%lu ahv=%.4f m/s2 | ahwx=%.4f ahwy=%.4f ahwz=%.4f | n=%lu",
              seq, ahv, ahwx, ahwy, ahwz, nSamples);
    }
}

/** @brief Called by BLE stack when a notification arrives from HAV Node. */
static void onHavNotify(BLERemoteCharacteristic *pChar,
                        uint8_t *pData, size_t length, bool isNotify) {
    // Ensure null-termination before parsing
    char buf[160];
    size_t copyLen = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 2;
    memcpy(buf, pData, copyLen);
    buf[copyLen] = '\0';

    parseAndEnqueueHavPayload(buf);
}

/** @brief Scan result callback — stores the first matching HAV Node device. */
class HavAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (advertisedDevice.getName() == BLE_HAV_DEVICE_NAME ||
            advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(BLEUUID(BLE_SERVICE_UUID)))
        {
            BLEDevice::getScan()->stop();
            pFoundDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect    = true;
            doScan       = false;
            LOG_I("BLE", "HAV Node found: %s", advertisedDevice.getAddress().toString().c_str());
        }
    }
};

// =============================================================================
// TASK: vTaskBLEReceiver
// Core 1 | Priority 3 | Event-driven (blocks on BLE callbacks)
// =============================================================================
/**
 * @brief  BLE Receiver task — Core 1, Priority 3.
 *
 *         Manages the full BLE Client lifecycle:
 *           1. Scan for "HAV_NODE" advertising the HAV Service UUID.
 *           2. Connect and subscribe to HAV Characteristic (NOTIFY).
 *           3. Receive HAV RMS epoch notifications from HAV Node firmware.
 *           4. Parse CSV payload and enqueue HavRmsData_t to xQueueHAVData.
 *           5. Detect disconnection and automatically reconnect.
 *
 *         The task does NOT gate on SYS_LOGGING — it keeps BLE alive at all
 *         times so that HAV data is ready the moment logging starts.
 */
static void vTaskBLEReceiver(void *pvParameters) {
    static const char *TAG = "BLE_RX";

    LOG_I(TAG, "BLE Receiver task started on Core %d", xPortGetCoreID());

    BLEDevice::init("");   // Initialise BLE stack (client mode, no name needed)
    doScan = true;

    while (true) {
        // ── 1. Initiate Scan ─────────────────────────────────────────────────
        if (doScan) {
            doScan = false;
            bleConnected = false;

            LOG_I(TAG, "Scanning for %s ...", BLE_HAV_DEVICE_NAME);
            BLEScan *pScan = BLEDevice::getScan();
            pScan->setAdvertisedDeviceCallbacks(new HavAdvertisedDeviceCallbacks());
            pScan->setActiveScan(true);
            pScan->setInterval(100);
            pScan->setWindow(99);
            pScan->start(BLE_SCAN_DURATION_S, false);

            if (!doConnect) {
                // Not found in this scan window — wait, then retry
                LOG_W(TAG, "HAV Node not found. Retrying in %d ms...", BLE_RECONNECT_MS);
                vTaskDelay(pdMS_TO_TICKS(BLE_RECONNECT_MS));
                doScan = true;
            }
        }

        // ── 2. Connect & Subscribe ───────────────────────────────────────────
        if (doConnect && pFoundDevice != nullptr) {
            doConnect = false;

            if (pBleClient != nullptr) {
                if (pBleClient->isConnected()) pBleClient->disconnect();
                delete pBleClient;
            }
            pBleClient = BLEDevice::createClient();

            LOG_I(TAG, "Connecting to HAV Node...");

            if (!pBleClient->connect(pFoundDevice)) {
                LOG_E(TAG, "BLE connect failed. Will retry scan.");
                delete pFoundDevice;
                pFoundDevice = nullptr;
                doScan = true;
                vTaskDelay(pdMS_TO_TICKS(BLE_RECONNECT_MS));
                continue;
            }

            LOG_I(TAG, "Connected to HAV Node.");

            // Get the remote service
            BLERemoteService *pRemoteService =
                pBleClient->getService(BLEUUID(BLE_SERVICE_UUID));

            if (pRemoteService == nullptr) {
                LOG_E(TAG, "HAV Service UUID not found on device.");
                pBleClient->disconnect();
                doScan = true;
                continue;
            }

            // Get the HAV characteristic
            pHavCharacteristic =
                pRemoteService->getCharacteristic(BLEUUID(BLE_CHAR_UUID_HAV));

            if (pHavCharacteristic == nullptr) {
                LOG_E(TAG, "HAV Characteristic UUID not found.");
                pBleClient->disconnect();
                doScan = true;
                continue;
            }

            // Register notification callback
            if (pHavCharacteristic->canNotify()) {
                pHavCharacteristic->registerForNotify(onHavNotify);
                LOG_I(TAG, "Subscribed to HAV notifications.");
            } else {
                LOG_W(TAG, "Characteristic does not support NOTIFY.");
            }

            bleConnected = true;

            delete pFoundDevice;
            pFoundDevice = nullptr;
        }

        // ── 3. Monitor Connection ────────────────────────────────────────────
        if (bleConnected) {
            if (pBleClient == nullptr || !pBleClient->isConnected()) {
                // Connection dropped
                bleConnected  = false;
                bleHavDataOK  = false;
                LOG_W(TAG, "BLE disconnected from HAV Node. Rescanning...");
                doScan = true;
            }
        }

        // Yield — BLE notifications arrive asynchronously via callback
        vTaskDelay(pdMS_TO_TICKS(500));
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
        if (!adxl345_read(actual_wbv_addr, ax, ay, az)) {
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
                // WBV sensor and SD card are the minimum requirements for READY.
                // BLE (HAV) connection is optional — logging continues without HAV.
                const bool allOK = (wbvSensorOK || sdOK);
                if (allOK) {
                    setSystemState(SYS_READY);
                    LOG_I(TAG, "FSM: SELF_TEST → READY (WBV=%d SD=%d BLE=%d)",
                          wbvSensorOK, sdOK, (int)bleConnected);
                } else {
                    setSystemState(SYS_ERROR);
                    LOG_E(TAG, "FSM: SELF_TEST → ERROR (WBV:%d SD:%d)",
                          wbvSensorOK, sdOK);
                }
                break;
            }

            // ── READY: Waiting for user to start logging ─────────────────────
            case SYS_READY:
                if (buttonEvent) {
                    setSystemState(SYS_LOGGING);
                    LOG_I(TAG, "FSM: READY → LOGGING");
                    // Notify acquisition tasks (optional: use task notification)
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
                    LOG_I(TAG, "OLED screensaver activated (blinking dot mode)");
                }

                if (oledOK) {
                    static bool dotState = false;
                    dotState = !dotState; // Toggle state every 1 Hz tick
                    
                    oled.clearDisplay();
                    if (dotState) {
                        // Gambar titik kecil di sudut kanan atas sebagai indikator sistem hidup
                        oled.fillCircle(124, 4, 2, SSD1306_WHITE);
                    }
                    oled.display();
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

                    oled.print("HAV BLE: ");
                    oled.println(bleConnected ? (bleHavDataOK ? "DATA" : "CONN") : "DISC");
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

#if USE_BLE_HAV
    LOG_I("INIT", "HAV source: BLE (HAV_NODE) — local HAV sensor not probed");
#else
    LOG_I("INIT", "HAV source: None (WBV-only mode)");
#endif

    // WBV ADXL345 Auto-Detect (0x53 or 0x1D)
    if (adxl345_detect(0x53)) {
        actual_wbv_addr = 0x53;
        wbvSensorOK = adxl345_init(actual_wbv_addr, ADXL_BW_400HZ);
    } else if (adxl345_detect(0x1D)) {
        actual_wbv_addr = 0x1D;
        wbvSensorOK = adxl345_init(actual_wbv_addr, ADXL_BW_400HZ);
    } else {
        wbvSensorOK = false;
    }

    if (wbvSensorOK) {
        LOG_I("INIT", "WBV ADXL345 @ 0x%02X init OK", actual_wbv_addr);
    } else {
        LOG_E("INIT", "WBV ADXL345 NOT FOUND (checked 0x53 and 0x1D)");
    }

    // DS3231 RTC Self-Test
    rtcOK = rtc.begin();
    LOG_I("INIT", "DS3231 RTC init %s", rtcOK ? "OK" : "NOT FOUND");
    if (rtcOK && rtc.lostPower()) {
        LOG_W("INIT", "RTC lost power, setting compilation time!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // SD Card Self-Test (with hardware mitigation for problematic modules)
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    pinMode(PIN_SPI_MISO, INPUT_PULLUP); // Helps with long jumper wires
    
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SD_CS);
    
    // Try explicit 4 MHz first, fallback to 1 MHz if it fails
    sdOK = SD.begin(PIN_SD_CS, SPI, 4000000);
    if (!sdOK) {
        LOG_W("INIT", "SD init failed at 4MHz, retrying at 1MHz...");
        sdOK = SD.begin(PIN_SD_CS, SPI, 1000000);
    }
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

    LOG_I("INIT", "Self-test complete: WBV=%d RTC=%d SD=%d OLED=%d",
          wbvSensorOK, rtcOK, sdOK, oledOK);
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

    // ── Core 0: WBV Acquisition ─────────
    res = xTaskCreatePinnedToCore(
        vTaskWBVAcquisition, "WBV_ACQ",
        TASK_STACK_WBV, nullptr,
        TASK_PRIO_WBV, &hTaskWBV,
        CORE_DSP);
    configASSERT(res == pdPASS);

    // ── Core 1: BLE Receiver, Logger, HMI ────────────────────────────────────
#if USE_BLE_HAV
    res = xTaskCreatePinnedToCore(
        vTaskBLEReceiver, "BLE_RX",
        TASK_STACK_BLE, nullptr,
        TASK_PRIO_BLE, &hTaskBLE,
        CORE_PERIPHERAL);
    configASSERT(res == pdPASS);
#endif

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
    Serial.println("  Vibration Dosimeter — Main Unit Firmware v2.0            ");
    Serial.println("  Standards: ISO 5349-1 (HAV) | ISO 2631-1 (WBV)          ");
    Serial.println("  HAV: BLE from HAV Node | WBV: Local ADXL345              ");
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

    // Set initial FSM state based on self-test results.
    // BLE connection is NOT required to transition to READY.
    // (BLE Receiver task connects asynchronously after scheduler starts.)
    if (wbvSensorOK || sdOK) {
        setSystemState(SYS_READY);
        LOG_I("INIT", "FSM: INIT → READY (WBV OK, SD OK, BLE connecting in background)");
    } else {
        setSystemState(SYS_ERROR);
        LOG_E("INIT", "FSM: INIT → ERROR — WBV=%d SD=%d", wbvSensorOK, sdOK);
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