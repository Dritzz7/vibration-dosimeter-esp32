/**
 * @file    main.cpp
 * @brief   Vibration Dosimeter ESP32 — HAV Node Firmware (Single-Core Polling)
 *
 * @details Hand-Arm Vibration (HAV) edge processing node conforming to ISO 5349-1.
 *
 *          ┌──────────────────────────────────────────────────────────────────┐
 *          │  Main Loop (Core 1)                                              │
 *          │   • High-frequency ADXL345 acquisition (3200 Hz via micro timer) │
 *          │   • ISO 5349-1 frequency weighting filter (Wh on X, Y, Z axes)   │
 *          │   • Accumulates 1-second RMS epoch (3200 samples)                 │
 *          │   • Transmits RMS results over BLE as GATT Server (Peripheral)    │
 *          └──────────────────────────────────────────────────────────────────┘
 *
 *          This unit operates as a BLE Server. The Main Unit subscribes to the HAV
 *          Characteristic (NOTIFY) to receive one-second epoch data.
 *          Payload format: "HAV,seq,millis_ms,ahwx,ahwy,ahwz,ahv,n_samples"
 *
 * @note    Target Hardware : ESP32 Dual-Core (240 MHz)
 *          Framework       : Arduino Core
 *          HAV Sensor      : ADXL345 (I2C, auto-detect 0x53 or 0x1D), 3200 Hz ODR
 *          BLE Role        : BLE Server ("HAV_NODE" device name)
 *
 * @author  Team EL4060
 * @date    2026-06-17
 */

// =============================================================================
// INCLUDES
// =============================================================================
#include <Arduino.h>
#include <Wire.h>

// BLE Server (ESP32 built-in Bluetooth stack)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Project filter coefficient headers (MATLAB-generated, ISO 8041)
#include "hav_coefficients.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =============================================================================
// DEBUG CONFIGURATION
// =============================================================================
#define DEBUG_ENABLED      1
#define SIMULATE_ADXL345   0

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
// --- I2C Bus (shared: HAV sensor) ---
#define PIN_I2C_SDA          21
#define PIN_I2C_SCL          22

// --- Battery ADC Monitoring (ADC1, safe with BLE) ---
#define ENABLE_BATTERY_MONITOR 1     ///< Set 1 when battery divider is connected to GPIO34, 0 for USB bench testing
#define PIN_VBAT_SENSE         34    ///< GPIO34 (ADC1_CH6) voltage divider
#define VBAT_DIVIDER_RATIO     2.0f  ///< R1=100k, R2=100k resistor divider
#define VBAT_MIN_VALID         2.80f ///< Minimum valid battery voltage (filters floating pin on USB)
#define VBAT_LOW_THRESHOLD     3.40f ///< Low battery cut-off warning (V)

// --- Mini RGB LED (KY-016 / SMD 0805) ---
// NOTE: GPIO 25 (DAC1/ADC2/RTC) conflicts with ESP32 BLE stack when BLE is active.
// GPIO 32 (ADC1_CH4) is BLE-safe and does not share RTC/DAC functions.
#define RGB_LED_PIN_RED      32     ///< GPIO 32 (ADC1 - BLE-safe) connected to Red channel
#define RGB_LED_PIN_GREEN    26     ///< GPIO 26 connected to Green channel
#define RGB_LED_PIN_BLUE     27     ///< GPIO 27 connected to Blue channel

/**
 * Hardware Polarity:
 * 1 = Common Cathode (Active HIGH: duty = val, pin HIGH = ON) - e.g. KY-016
 * 0 = Common Anode   (Active LOW : duty = 255 - val, pin LOW = ON)
 */
#define RGB_LED_COMMON_CATHODE 1

#define RGB_LED_MAX_BRIGHTNESS  255   ///< Full 100% duty cycle (255) for clear visibility

// =============================================================================
// HAV FSM STATE ENUMERATION
// =============================================================================
typedef enum {
    HAV_STATE_INIT = 0,            ///< Power-on / booting
    HAV_STATE_BLE_ADVERTISING,     ///< BLE advertising (waiting for Main Unit)
    HAV_STATE_LOGGING_NORMAL,      ///< Connected, active logging
    HAV_STATE_COMM_LOST,           ///< BLE disconnected during session
    HAV_STATE_ERROR,               ///< Sensor or hardware fault
    HAV_STATE_LOW_BATTERY          ///< Battery critical (V_bat < 3.4V)
} hav_fsm_state_t;

// Shorthand aliases
#define STATE_INIT                 HAV_STATE_INIT
#define STATE_BLE_ADVERTISING      HAV_STATE_BLE_ADVERTISING
#define STATE_LOGGING_NORMAL       HAV_STATE_LOGGING_NORMAL
#define STATE_COMM_LOST            HAV_STATE_COMM_LOST
#define STATE_ERROR                HAV_STATE_ERROR
#define STATE_LOW_BATTERY          HAV_STATE_LOW_BATTERY

// =============================================================================
// ADXL345 REGISTER MAP & CONFIG
// =============================================================================
#define ADXL345_ADDR_1       0x53   // Primary I2C address (SDO tied to GND)
#define ADXL345_ADDR_2       0x1D   // Secondary I2C address (SDO tied to VCC)

#define REG_DEVID            0x00   // Device ID Register
#define REG_BW_RATE          0x2C   // Data rate and power control
#define REG_POWER_CTL        0x2D   // Power-saving features control
#define REG_DATA_FORMAT      0x31   // Data format control
#define REG_DATAX0           0x32   // X-Axis Data 0 (start register for tri-axial read)

#define ADXL345_DEVID_VALUE  0xE5   // Expected Device ID for ADXL345

// ADXL345 settings
#define ADXL_BW_3200HZ       0x0F   // 3200 Hz ODR — HAV
#define ADXL_FORMAT_FULLRES_16G 0x0B // FULL_RES | Range ±16g

#define ADXL_SCALE_G_PER_LSB 0.0039f  // Scale factor in full-resolution mode (g/LSB)
#define G_TO_MPS2            9.80665f // Gravitational constant (m/s²)

// =============================================================================
// BLE SERVER CONFIGURATION
// =============================================================================
#define BLE_DEVICE_NAME      "HAV_NODE"
#define BLE_SERVICE_UUID     "9b6f0001-5f5a-4f0d-9d7f-000000000001"
#define BLE_CHAR_UUID_HAV    "9b6f0002-5f5a-4f0d-9d7f-000000000002"

// =============================================================================
// SAMPLING & TIMING CONSTANTS
// =============================================================================
const float FS = FS_HAV;                     ///< 3200 Hz sampling rate (from hav_coefficients.h)

// 1 / 3200 Hz = 312.5 us period.
// Multiplied by 100 to represent 312.5 us precisely as an integer (31250) for micros() usage.
const uint32_t SAMPLE_PERIOD_US_X100 = 31250;
const uint16_t HAV_EPOCH_SAMPLES = 3200;     ///< Epoch window of 1 second (3200 samples)

// =============================================================================
// GLOBAL STATE
// =============================================================================
uint8_t adxlAddress = 0x00;              ///< Detected ADXL345 I2C address
bool adxlAvailable = false;             ///< True if ADXL345 is successfully initialized

BLECharacteristic *havCharacteristic = nullptr; ///< Characteristic for sending HAV data
bool bleClientConnected = false;        ///< Connection state with BLE Client

uint32_t packetCounter = 0;             ///< Sequential packet counter for transmitted epochs

// =============================================================================
// BIQUAD CASCADE FILTER CLASS
// =============================================================================
/**
 * @class BiquadCascade
 * @brief Thread-local Direct Form II Transposed IIR biquad cascade filter.
 *        Each filter instance contains its own coefficients and delay line states.
 *
 * Difference equation per section (a0 = 1 normalised):
 *   y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] − a1·y[n-1] − a2·y[n-2]
 *
 * Coefficient layout: coeff[section][5] = {b0, b1, b2, a1, a2}
 */
class BiquadCascade {
public:
    static constexpr int MAX_SECTIONS = 4; ///< Maximum supported biquad sections

    /**
     * @brief Construct a new Biquad Cascade filter instance.
     * @param coeff        Pointer to the 2D array of coefficients.
     * @param numSections  Number of active biquad sections in the cascade.
     */
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

    /** @brief Reset all internal delay-line states to zero. */
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

// --- ISO 5349-1 Wh Weighting Filters (one for each axis) ---
BiquadCascade filterX(coeff_wh, NUM_SECTIONS_WH);
BiquadCascade filterY(coeff_wh, NUM_SECTIONS_WH);
BiquadCascade filterZ(coeff_wh, NUM_SECTIONS_WH);

// =============================================================================
// I2C HELPER FUNCTIONS (Wire-based, blocking)
// =============================================================================
/**
 * @brief Write a single byte value to an I2C device register.
 * @param addr   I2C slave address.
 * @param reg    Register address to write to.
 * @param value  Byte value to write.
 * @return true on success, false on failure.
 */
bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

/**
 * @brief Read a single byte value from an I2C device register.
 * @param addr        I2C slave address.
 * @param reg         Register address to read from.
 * @param[out] value  Reference to store the read byte.
 * @return true on success, false on failure.
 */
bool readRegister(uint8_t addr, uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom((int)addr, 1) != 1 || Wire.available() < 1) {
        return false;
    }

    value = Wire.read();
    return true;
}

/**
 * @brief Perform a burst read of multiple consecutive registers over I2C.
 * @param addr         I2C slave address.
 * @param startReg     Starting register address.
 * @param[out] buffer  Buffer to store the read data.
 * @param length       Number of bytes to read.
 * @return true on success, false on failure.
 */
bool readMultipleRegisters(uint8_t addr, uint8_t startReg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(addr);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if ((uint8_t)Wire.requestFrom((int)addr, (int)length) != length) {
        while (Wire.available()) {
            Wire.read(); // Flush any garbage
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

// =============================================================================
// ADXL345 DRIVER FUNCTIONS
// =============================================================================
/**
 * @brief Scan for ADXL345 sensor address (0x53 or 0x1D) and verify Device ID.
 *        Includes full I2C bus scan for hardware troubleshooting.
 * @return true if detected, false otherwise.
 */
bool detectADXL345() {
    LOG_I("I2C_SCAN", "Scanning I2C bus (SDA=GPIO%d, SCL=GPIO%d)...", PIN_I2C_SDA, PIN_I2C_SCL);
    uint8_t devicesFound = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            LOG_I("I2C_SCAN", "  -> I2C device found at address 0x%02X", addr);
            devicesFound++;
        }
    }

    if (devicesFound == 0) {
        LOG_E("I2C_SCAN", "No I2C devices found on bus! Check SDA/SCL wiring and VCC power.");
    }

    uint8_t devid = 0;

    // Check primary address 0x53
    if (readRegister(ADXL345_ADDR_1, REG_DEVID, devid)) {
        LOG_I("INIT", "Found device at 0x53, REG_DEVID = 0x%02X (Expected 0xE5)", devid);
        if (devid == ADXL345_DEVID_VALUE) {
            adxlAddress = ADXL345_ADDR_1;
            return true;
        }
    }

    // Check secondary address 0x1D
    if (readRegister(ADXL345_ADDR_2, REG_DEVID, devid)) {
        LOG_I("INIT", "Found device at 0x1D, REG_DEVID = 0x%02X (Expected 0xE5)", devid);
        if (devid == ADXL345_DEVID_VALUE) {
            adxlAddress = ADXL345_ADDR_2;
            return true;
        }
    }

    adxlAddress = 0x00;
    return false;
}

/**
 * @brief Initialize and configure the ADXL345 sensor for 3200 Hz ODR / ±16g.
 * @return true on successful configuration.
 */
bool setupADXL345() {
    // Pre-check line states for hardware diagnostic
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    delayMicroseconds(50);
    int sdaVal = digitalRead(PIN_I2C_SDA);
    int sclVal = digitalRead(PIN_I2C_SCL);

    LOG_I("I2C_SCAN", "Idle I2C Line States: SDA(GPIO%d)=%s, SCL(GPIO%d)=%s",
          PIN_I2C_SDA, sdaVal ? "HIGH (OK)" : "LOW (SHORT/GROUNDED!)",
          PIN_I2C_SCL, sclVal ? "HIGH (OK)" : "LOW (SHORT/GROUNDED!)");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000); // Start with 100 kHz I2C clock for initialization

    if (!detectADXL345()) {
        return false;
    }

    LOG_I("INIT", "ADXL345 HAV detected at I2C address 0x%02X", adxlAddress);

    // 1. Enter Standby mode to configure
    if (!writeRegister(adxlAddress, REG_POWER_CTL, 0x00)) {
        return false;
    }

    // 2. Set output data rate to 3200 Hz ODR
    if (!writeRegister(adxlAddress, REG_BW_RATE, ADXL_BW_3200HZ)) {
        return false;
    }

    // 3. Set data format: Full resolution mode, Range ±16g
    if (!writeRegister(adxlAddress, REG_DATA_FORMAT, ADXL_FORMAT_FULLRES_16G)) {
        return false;
    }

    // 4. Enter Measurement mode
    if (!writeRegister(adxlAddress, REG_POWER_CTL, 0x08)) {
        return false;
    }

    Wire.setClock(400000); // Raise I2C clock to 400 kHz for high-frequency polling

    LOG_I("INIT", "ADXL345 HAV configuration complete:");
    LOG_I("INIT", "  - Range    : +/-16g full resolution");
    LOG_I("INIT", "  - DataRate : 3200 Hz");

    return true;
}

/**
 * @brief Read tri-axial acceleration from the configured ADXL345 sensor.
 * @param[out] ax  Acceleration on X-axis [m/s²]
 * @param[out] ay  Acceleration on Y-axis [m/s²]
 * @param[out] az  Acceleration on Z-axis [m/s²]
 * @return true on success.
 */
bool readADXL345(float &ax, float &ay, float &az) {
    if (!adxlAvailable) {
        return false;
    }

    uint8_t data[6];
    if (!readMultipleRegisters(adxlAddress, REG_DATAX0, data, 6)) {
        return false;
    }

    const int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
    const int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    const int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);

    ax = rawX * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    ay = rawY * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    az = rawZ * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;

    return true;
}

// =============================================================================
// MINI RGB LED SUBSYSTEM (LEDC PWM & FREERTOS BACKGROUND TASK)
// =============================================================================
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

static const rgb_color_t COLOR_OFF    = {0,   0,   0};
static const rgb_color_t COLOR_RED    = {255, 0,   0};
static const rgb_color_t COLOR_GREEN  = {0,   255, 0};
static const rgb_color_t COLOR_BLUE   = {0,   0,   255};

static volatile hav_fsm_state_t s_current_led_state = HAV_STATE_INIT;
static SemaphoreHandle_t        s_mutex_led         = nullptr;
static TaskHandle_t             s_task_led_handle   = nullptr;
static bool                     s_led_initialized   = false;

static inline uint32_t scale_led_duty(uint8_t val) {
    uint32_t scaled = ((uint32_t)val * (uint32_t)RGB_LED_MAX_BRIGHTNESS) / 255U;
#if RGB_LED_COMMON_CATHODE
    return scaled;
#else
    return 255U - scaled;
#endif
}

static void apply_hw_color(uint8_t r, uint8_t g, uint8_t b) {
    analogWrite(RGB_LED_PIN_RED,   scale_led_duty(r));
    analogWrite(RGB_LED_PIN_GREEN, scale_led_duty(g));
    analogWrite(RGB_LED_PIN_BLUE,  scale_led_duty(b));
}

static void vTaskRgbLedPattern(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20); // 50 Hz update rate
    uint32_t tickCount = 0;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        tickCount++;

        hav_fsm_state_t state = HAV_STATE_INIT;
        if (s_mutex_led && xSemaphoreTake(s_mutex_led, pdMS_TO_TICKS(5)) == pdTRUE) {
            state = s_current_led_state;
            xSemaphoreGive(s_mutex_led);
        } else {
            state = s_current_led_state;
        }

        switch (state) {
            case HAV_STATE_INIT:
            case HAV_STATE_BLE_ADVERTISING: {
                // Blue color, slow blink (1 Hz, 50% duty: 500 ms ON, 500 ms OFF)
                const uint32_t phase = tickCount % 50;
                if (phase < 25) apply_hw_color(COLOR_BLUE.r, COLOR_BLUE.g, COLOR_BLUE.b);
                else            apply_hw_color(COLOR_OFF.r, COLOR_OFF.g, COLOR_OFF.b);
                break;
            }

            case HAV_STATE_LOGGING_NORMAL: {
                // Skenario B: Green color, smooth breathing over 2.0 seconds
                const uint32_t phase = tickCount % 100;
                const float rad = (float)phase * (2.0f * (float)M_PI / 100.0f);
                const float normalized = (1.0f - cosf(rad)) * 0.5f;
                const uint8_t green_val = (uint8_t)(35.0f + normalized * (255.0f - 35.0f));
                apply_hw_color(0, green_val, 0);
                break;
            }

            case HAV_STATE_COMM_LOST:
            case HAV_STATE_ERROR: {
                // Skenario D & E: Red color, alternating blink at 2 Hz (250 ms ON, 250 ms OFF)
                const uint32_t phase = tickCount % 25;
                if (phase < 13) apply_hw_color(COLOR_RED.r, COLOR_RED.g, COLOR_RED.b);
                else            apply_hw_color(COLOR_OFF.r, COLOR_OFF.g, COLOR_OFF.b);
                break;
            }

            case HAV_STATE_LOW_BATTERY: {
                // Double-blink RED every 3 seconds
                const uint32_t phase = tickCount % 150;
                if (phase < 5 || (phase >= 10 && phase < 15)) {
                    apply_hw_color(COLOR_RED.r, COLOR_RED.g, COLOR_RED.b);
                } else {
                    apply_hw_color(COLOR_OFF.r, COLOR_OFF.g, COLOR_OFF.b);
                }
                break;
            }

            default:
                apply_hw_color(COLOR_OFF.r, COLOR_OFF.g, COLOR_OFF.b);
                break;
        }
    }
}

esp_err_t rgb_led_init(void) {
    if (s_led_initialized) return ESP_OK;

    if (!s_mutex_led) {
        s_mutex_led = xSemaphoreCreateMutex();
        if (!s_mutex_led) return ESP_ERR_NO_MEM;
    }

    pinMode(RGB_LED_PIN_RED,   OUTPUT);
    pinMode(RGB_LED_PIN_GREEN, OUTPUT);
    pinMode(RGB_LED_PIN_BLUE,  OUTPUT);

    // Initial state: OFF (100% non-blocking, zero delay)
    apply_hw_color(0, 0, 0);

    BaseType_t res = xTaskCreatePinnedToCore(
        vTaskRgbLedPattern, "RGB_LED_FSM",
        4096, nullptr, 1, &s_task_led_handle, 1
    );

    if (res != pdPASS) return ESP_FAIL;

    s_led_initialized = true;
    s_current_led_state = HAV_STATE_BLE_ADVERTISING;
    return ESP_OK;
}

void rgb_led_set_state(hav_fsm_state_t new_state) {
    if (!s_led_initialized && rgb_led_init() != ESP_OK) return;

    if (s_mutex_led && xSemaphoreTake(s_mutex_led, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_current_led_state != new_state) {
            const char *state_str = "UNKNOWN";
            switch (new_state) {
                case HAV_STATE_INIT:            state_str = "INIT (Blue 1Hz)"; break;
                case HAV_STATE_BLE_ADVERTISING: state_str = "BLE_ADVERTISING (Blue 1Hz)"; break;
                case HAV_STATE_LOGGING_NORMAL:  state_str = "LOGGING_NORMAL (Green Breathe)"; break;
                case HAV_STATE_COMM_LOST:       state_str = "COMM_LOST (Red 2Hz)"; break;
                case HAV_STATE_ERROR:           state_str = "ERROR (Red 2Hz)"; break;
                case HAV_STATE_LOW_BATTERY:     state_str = "LOW_BATTERY (Red Double-Blink)"; break;
            }
            LOG_I("LED", "FSM State transition: -> %s", state_str);
            s_current_led_state = new_state;
        }
        xSemaphoreGive(s_mutex_led);
    } else {
        s_current_led_state = new_state;
    }
}

hav_fsm_state_t rgb_led_get_state(void) {
    hav_fsm_state_t st = HAV_STATE_INIT;
    if (s_mutex_led && xSemaphoreTake(s_mutex_led, pdMS_TO_TICKS(5)) == pdTRUE) {
        st = s_current_led_state;
        xSemaphoreGive(s_mutex_led);
    } else {
        st = s_current_led_state;
    }
    return st;
}


void rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    apply_hw_color(r, g, b);
}

void rgb_led_off(void) {
    apply_hw_color(COLOR_OFF.r, COLOR_OFF.g, COLOR_OFF.b);
}

// =============================================================================
// BLE SERVER CALLBACKS
// =============================================================================
/**
 * @class HavBleServerCallbacks
 * @brief BLE Server Callbacks to monitor client connections and handle advertising.
 */
class HavBleServerCallbacks : public BLEServerCallbacks {
    /** @brief Triggered when a BLE Client connects. */
    void onConnect(BLEServer *server) override {
        bleClientConnected = true;
        LOG_I("BLE", "BLE client connected. Transitioning LED to LOGGING_NORMAL.");
        rgb_led_set_state(STATE_LOGGING_NORMAL);
    }

    /** @brief Triggered when a BLE Client disconnects; restarts advertising. */
    void onDisconnect(BLEServer *server) override {
        bleClientConnected = false;
        LOG_W("BLE", "BLE client disconnected. Transitioning LED to COMM_LOST & restarting advertising...");
        rgb_led_set_state(STATE_COMM_LOST);
        server->getAdvertising()->start();
    }
};

/**
 * @brief Sample battery voltage through resistive divider on ADC1.
 * @return Battery voltage in Volts.
 */
static float readBatteryVoltage() {
    const uint16_t raw = analogRead(PIN_VBAT_SENSE);
    const float pinVoltage = (raw / 4095.0f) * 3.3f;
    return pinVoltage * VBAT_DIVIDER_RATIO;
}

// =============================================================================
// BLE SETUP
// =============================================================================
/**
 * @brief Initialize the BLE Stack, GATT Server, Service, and Characteristic.
 */
void setupBLE() {
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(128);

    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new HavBleServerCallbacks());

    BLEService *service = server->createService(BLE_SERVICE_UUID);

    havCharacteristic = service->createCharacteristic(
        BLE_CHAR_UUID_HAV,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    // Client Characteristic Configuration Descriptor (needed for Notifications)
    havCharacteristic->addDescriptor(new BLE2902());

    // Set fallback initial value
    havCharacteristic->setValue("HAV,0,0,0,0,0,0,0");

    service->start();

    // Start BLE Advertising
    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    advertising->start();

    LOG_I("BLE", "BLE HAV Node advertising started.");
}

// =============================================================================
// SEND HAV BLE PACKET
// =============================================================================
/**
 * @brief Pack aggregate data and notify the subscribed BLE client.
 *
 *        Payload Format (CSV):
 *        "HAV,<seq>,<millis_ms>,<ahwx>,<ahwy>,<ahwz>,<ahv>,<n_samples>"
 *
 * @param ahwx      Wh-weighted RMS acceleration on X-axis [m/s²]
 * @param ahwy      Wh-weighted RMS acceleration on Y-axis [m/s²]
 * @param ahwz      Wh-weighted RMS acceleration on Z-axis [m/s²]
 * @param ahv       Vector-sum RMS acceleration [m/s²]
 * @param nSamples  Number of samples in this epoch
 */
void sendHavBlePacket(float ahwx, float ahwy, float ahwz, float ahv, uint16_t nSamples) {
    char payload[128];
    const uint32_t nowMs = millis();

    snprintf(payload, sizeof(payload),
             "HAV,%lu,%lu,%.4f,%.4f,%.4f,%.4f,%u",
             packetCounter,
             nowMs,
             ahwx,
             ahwy,
             ahwz,
             ahv,
             nSamples);

    // Print raw payload to serial console regardless of connection state
    Serial.println(payload);

    if (havCharacteristic != nullptr) {
        havCharacteristic->setValue(payload);
        if (bleClientConnected) {
            havCharacteristic->notify();
        }
    }

    packetCounter++;
}

// =============================================================================
// ARDUINO SETUP — Runs once on Core 1 before the loop
// =============================================================================
void setup() {

    // Disable brownout reset
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

#if DEBUG_ENABLED
    Serial.begin(115200);
    delay(500); // Short settling delay
#endif

    Serial.println();
    Serial.println("==========================================================");
    Serial.println("  Vibration Dosimeter — HAV Node Firmware v2.0             ");
    Serial.println("  Standards: ISO 5349-1 (HAV)                             ");
    Serial.println("  Peripherals: ADXL345 + Wh Filter + BLE Server            ");
    Serial.println("  Design Configuration: No RTC, no SD, no OLED             ");
    Serial.println("==========================================================");

    // Initialize Mini RGB LED module (KY-016 / SMD 0805)
    if (rgb_led_init() == ESP_OK) {
        LOG_I("INIT", "Mini RGB LED initialized on GPIO R:%d G:%d B:%d",
              RGB_LED_PIN_RED, RGB_LED_PIN_GREEN, RGB_LED_PIN_BLUE);
        rgb_led_set_state(STATE_INIT);
    } else {
        LOG_E("INIT", "Mini RGB LED initialization failed!");
    }

#if SIMULATE_ADXL345
    adxlAvailable = true;
    LOG_I("INIT", "ADXL345 HAV is SIMULATED");
#else
    adxlAvailable = setupADXL345();
#endif

    if (!adxlAvailable) {
        LOG_E("INIT", "ADXL345 HAV sensor initialization FAILED.");
        rgb_led_set_state(STATE_ERROR);
        LOG_W("INIT", "Check wiring connections:");
        LOG_W("INIT", "  - VCC -> 3V3");
        LOG_W("INIT", "  - GND -> GND");
        LOG_W("INIT", "  - SDA -> GPIO21");
        LOG_W("INIT", "  - SCL -> GPIO22");
        LOG_W("INIT", "  - CS  -> 3V3");
        LOG_W("INIT", "  - SDO -> GND (for 0x53) or 3V3 (for 0x1D)");
    } else {
        rgb_led_set_state(STATE_BLE_ADVERTISING);
    }

    setupBLE();

    LOG_I("INIT", "Sampling rate HAV set to %.1f Hz", FS);
    LOG_I("INIT", "Expected BLE CSV Packet Format:");
    LOG_I("INIT", "  HAV,seq,millis_ms,ahwx,ahwy,ahwz,ahv,n_samples");
}

// =============================================================================
// ARDUINO LOOP
// =============================================================================
/**
 * @brief High-frequency main acquisition and filtering loop.
 *        Executes at exactly 3200 Hz using micros() timing control.
 */
void loop() {
    static uint64_t nextSampleTimeUsX100 = (uint64_t)micros() * 100ULL + SAMPLE_PERIOD_US_X100;

    static float sumX2 = 0.0f;
    static float sumY2 = 0.0f;
    static float sumZ2 = 0.0f;
    static uint16_t sampleCount = 0;

    static float lastRawX = 0.0f;
    static float lastRawY = 0.0f;
    static float lastRawZ = 0.0f;

    static uint32_t lastErrorPrintMs = 0;

    const uint64_t nowUsX100 = (uint64_t)micros() * 100ULL;

    if (nowUsX100 >= nextSampleTimeUsX100) {
        nextSampleTimeUsX100 += SAMPLE_PERIOD_US_X100;

        float ax = 0.0f, ay = 0.0f, az = 0.0f;

        // ── Sensor Read ──────────────────────────────────────────────────────
#if SIMULATE_ADXL345
        // Generate a 20 Hz sine wave for simulated HAV data, amplitude 1.5 m/s²
        const float t = (float)nowUsX100 / 1000000.0f;
        ax = 1.5f * sinf(2.0f * PI * 20.0f * t);
        ay = 0.5f * cosf(2.0f * PI * 15.0f * t);
        az = 0.2f * sinf(2.0f * PI * 10.0f * t);
#else
        if (!readADXL345(ax, ay, az)) {
            const uint32_t nowMs = millis();
            if (nowMs - lastErrorPrintMs >= 1000) {
                lastErrorPrintMs = nowMs;
                LOG_E("SENSOR", "ADXL345 HAV read failed! Retrying initialization...");
                rgb_led_set_state(STATE_ERROR);

                // Attempt hardware sensor recovery
                Wire.setClock(100000);
                adxlAvailable = setupADXL345();
                Wire.setClock(400000);

                if (adxlAvailable) {
                    rgb_led_set_state(bleClientConnected ? STATE_LOGGING_NORMAL : STATE_BLE_ADVERTISING);
                }
            }
            return;
        }
#endif

        lastRawX = ax;
        lastRawY = ay;
        lastRawZ = az;

        // ── Frequency Weighting: ISO 5349-1 Wh Weighting ─────────────────────
        const float axWh = filterX.process(ax);
        const float ayWh = filterY.process(ay);
        const float azWh = filterZ.process(az);

        // ── RMS Accumulation ─────────────────────────────────────────────────
        sumX2 += axWh * axWh;
        sumY2 += ayWh * ayWh;
        sumZ2 += azWh * azWh;
        sampleCount++;

        // ── 1-Second Epoch Boundary ──────────────────────────────────────────
        if (sampleCount >= HAV_EPOCH_SAMPLES) {
            const float ahwx = sqrtf(sumX2 / sampleCount);
            const float ahwy = sqrtf(sumY2 / sampleCount);
            const float ahwz = sqrtf(sumZ2 / sampleCount);

            // ISO 5349-1 vector-sum: ahv = √(ahwx² + ahwy² + ahwz²)
            const float ahv = sqrtf(ahwx * ahwx + ahwy * ahwy + ahwz * ahwz);

            // Log raw axes data alongside the BLE transmission info
            Serial.printf("RAW: ax=%.6f, ay=%.6f, az=%.6f | ", lastRawX, lastRawY, lastRawZ);

            sendHavBlePacket(ahwx, ahwy, ahwz, ahv, sampleCount);


#if ENABLE_BATTERY_MONITOR
            // ── Periodic battery voltage monitoring (every 5 seconds) ─────────
            static uint32_t lastBatteryCheckMs = 0;
            const uint32_t nowMs = millis();
            if (nowMs - lastBatteryCheckMs >= 5000) {
                lastBatteryCheckMs = nowMs;
                const float vbat = readBatteryVoltage();
                if (vbat >= VBAT_MIN_VALID && vbat < VBAT_LOW_THRESHOLD) {
                    LOG_W("PWR", "Low battery detected: %.2f V < %.2f V", vbat, VBAT_LOW_THRESHOLD);
                    rgb_led_set_state(STATE_LOW_BATTERY);
                }
            }
#endif

            // Reset accumulators
            sumX2 = 0.0f;
            sumY2 = 0.0f;
            sumZ2 = 0.0f;
            sampleCount = 0;
        }
    }
}