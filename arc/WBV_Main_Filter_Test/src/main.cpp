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

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <math.h>

#include "wbv_coefficients.h"

// =======================================================
// DEBUG
// =======================================================
#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
  #define LOG_I(tag, fmt, ...) Serial.printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
  #define LOG_W(tag, fmt, ...) Serial.printf("[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
  #define LOG_E(tag, fmt, ...) Serial.printf("[ERR ][%s] " fmt "\n", tag, ##__VA_ARGS__)
#else
  #define LOG_I(tag, fmt, ...) do {} while (0)
  #define LOG_W(tag, fmt, ...) do {} while (0)
  #define LOG_E(tag, fmt, ...) do {} while (0)
#endif

// =======================================================
// PIN DEFINITIONS
// =======================================================
#define PIN_I2C_SDA          21
#define PIN_I2C_SCL          22

#define PIN_SPI_MOSI         23
#define PIN_SPI_MISO         19
#define PIN_SPI_CLK          18
#define PIN_SD_CS             5

#define PIN_BUTTON            4
#define PIN_LED_STATUS        2

// =======================================================
// ADXL345 REGISTER MAP
// =======================================================
#define ADXL345_ADDR_1       0x53
#define ADXL345_ADDR_2       0x1D

#define ADXL_REG_DEVID       0x00
#define ADXL_REG_BW_RATE     0x2C
#define ADXL_REG_POWER_CTL   0x2D
#define ADXL_REG_DATA_FORMAT 0x31
#define ADXL_REG_DATAX0      0x32

#define ADXL_DEVID_EXPECTED  0xE5
#define ADXL_BW_400HZ        0x0C
#define ADXL_FORMAT_FULLRES_16G 0x0B

#define ADXL_SCALE_G_PER_LSB 0.0039f
#define G_TO_MPS2            9.80665f

// =======================================================
// TIMING
// =======================================================
#define WBV_SAMPLE_RATE_HZ   400U
#define WBV_PERIOD_US        2500U
#define WBV_EPOCH_SAMPLES    400U

#define LOGGER_PERIOD_MS     1000U
#define HMI_PERIOD_MS        100U
#define OLED_UPDATE_DIVIDER  10U
#define BUTTON_DEBOUNCE_MS   50U

// =======================================================
// RTOS CONFIG
// =======================================================
#define TASK_STACK_WBV       4096U
#define TASK_STACK_LOGGER    8192U
#define TASK_STACK_HMI       4096U

#define TASK_PRIO_WBV        5
#define TASK_PRIO_LOGGER     3
#define TASK_PRIO_HMI        2

#define CORE_DSP             0
#define CORE_PERIPHERAL      1

#define QUEUE_WBV_LENGTH     8U

// =======================================================
// DATA STRUCTURE
// =======================================================
typedef struct {
    float rawAx;
    float rawAy;
    float rawAz;
    float awx;
    float awy;
    float awz;
    float av;
    uint32_t timestamp;
    uint32_t n_samples;
} WbvRmsData_t;

// =======================================================
// SYSTEM STATE
// =======================================================
typedef enum {
    SYS_INIT = 0,
    SYS_READY,
    SYS_LOGGING,
    SYS_ERROR
} SystemState_t;

// =======================================================
// GLOBAL RTOS HANDLES
// =======================================================
static TaskHandle_t hTaskWBV = nullptr;
static TaskHandle_t hTaskLogger = nullptr;
static TaskHandle_t hTaskHMI = nullptr;

static QueueHandle_t xQueueWBVData = nullptr;

static SemaphoreHandle_t xMutexI2C = nullptr;
static SemaphoreHandle_t xMutexSD = nullptr;
static SemaphoreHandle_t xMutexState = nullptr;

static volatile SystemState_t systemState = SYS_INIT;

// =======================================================
// GLOBAL PERIPHERAL STATE
// =======================================================
static bool wbvSensorOK = false;
static bool rtcOK = false;
static bool sdOK = false;
static bool oledOK = false;

static uint8_t adxlAddress = 0x00;

static RTC_DS3231 rtc;
static Adafruit_SSD1306 oled(128, 64, &Wire, -1);

static WbvRmsData_t latestWbvData = {};
static bool latestWbvValid = false;

// =======================================================
// BIQUAD CASCADE FILTER
// =======================================================
class BiquadCascade {
public:
    static constexpr int MAX_SECTIONS = 4;

    BiquadCascade(const float (*coeff)[5], int numSections)
        : coeff_(coeff), numSections_(numSections) {
        reset();
    }

    inline float process(float input) {
        float x = input;

        for (int i = 0; i < numSections_; i++) {
            const float b0 = coeff_[i][0];
            const float b1 = coeff_[i][1];
            const float b2 = coeff_[i][2];
            const float a1 = coeff_[i][3];
            const float a2 = coeff_[i][4];

            const float y = b0 * x + b1 * x1_[i] + b2 * x2_[i]
                          - a1 * y1_[i] - a2 * y2_[i];

            x2_[i] = x1_[i];
            x1_[i] = x;
            y2_[i] = y1_[i];
            y1_[i] = y;
            x = y;
        }

        return x;
    }

    void reset() {
        for (int i = 0; i < MAX_SECTIONS; i++) {
            x1_[i] = 0.0f;
            x2_[i] = 0.0f;
            y1_[i] = 0.0f;
            y2_[i] = 0.0f;
        }
    }

private:
    const float (*coeff_)[5];
    int numSections_;
    float x1_[MAX_SECTIONS];
    float x2_[MAX_SECTIONS];
    float y1_[MAX_SECTIONS];
    float y2_[MAX_SECTIONS];
};

// =======================================================
// SYSTEM STATE HELPERS
// =======================================================
static SystemState_t getSystemState() {
    SystemState_t s = systemState;

    if (xMutexState && xSemaphoreTake(xMutexState, pdMS_TO_TICKS(5)) == pdTRUE) {
        s = systemState;
        xSemaphoreGive(xMutexState);
    }

    return s;
}

static void setSystemState(SystemState_t newState) {
    if (xMutexState && xSemaphoreTake(xMutexState, pdMS_TO_TICKS(10)) == pdTRUE) {
        systemState = newState;
        xSemaphoreGive(xMutexState);
    } else {
        systemState = newState;
    }
}

static const char* stateToString(SystemState_t s) {
    switch (s) {
        case SYS_INIT:    return "INIT";
        case SYS_READY:   return "READY";
        case SYS_LOGGING: return "LOGGING";
        case SYS_ERROR:   return "ERROR";
        default:          return "UNKNOWN";
    }
}

// =======================================================
// I2C HELPERS
// =======================================================
static bool i2c_writeReg(uint8_t devAddr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool i2c_readReg(uint8_t devAddr, uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom((int)devAddr, 1) != 1) {
        return false;
    }

    if (!Wire.available()) {
        return false;
    }

    value = Wire.read();
    return true;
}

static bool i2c_readBurst(uint8_t devAddr, uint8_t startReg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(devAddr);
    Wire.write(startReg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if ((uint8_t)Wire.requestFrom((int)devAddr, (int)len) != len) {
        while (Wire.available()) {
            Wire.read();
        }
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) {
            return false;
        }
        buf[i] = Wire.read();
    }

    return true;
}

// =======================================================
// ADXL345 DRIVER
// =======================================================
static bool adxl345_detectAddress(uint8_t addr) {
    uint8_t devid = 0;

    if (!i2c_readReg(addr, ADXL_REG_DEVID, devid)) {
        return false;
    }

    return devid == ADXL_DEVID_EXPECTED;
}

static bool adxl345_init(uint8_t addr) {
    if (!i2c_writeReg(addr, ADXL_REG_POWER_CTL, 0x00)) return false;
    if (!i2c_writeReg(addr, ADXL_REG_BW_RATE, ADXL_BW_400HZ)) return false;
    if (!i2c_writeReg(addr, ADXL_REG_DATA_FORMAT, ADXL_FORMAT_FULLRES_16G)) return false;
    if (!i2c_writeReg(addr, ADXL_REG_POWER_CTL, 0x08)) return false;
    return true;
}

static bool adxl345_autoDetectAndInit() {
    bool ok = false;

    if (xSemaphoreTake(xMutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        Wire.setClock(100000);

        if (adxl345_detectAddress(ADXL345_ADDR_1)) {
            adxlAddress = ADXL345_ADDR_1;
            ok = adxl345_init(adxlAddress);
        } else if (adxl345_detectAddress(ADXL345_ADDR_2)) {
            adxlAddress = ADXL345_ADDR_2;
            ok = adxl345_init(adxlAddress);
        }

        Wire.setClock(400000);
        xSemaphoreGive(xMutexI2C);
    }

    return ok;
}

static bool adxl345_read(float &ax, float &ay, float &az) {
    if (!wbvSensorOK) {
        return false;
    }

    uint8_t raw[6];

    if (xSemaphoreTake(xMutexI2C, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }

    const bool ok = i2c_readBurst(adxlAddress, ADXL_REG_DATAX0, raw, 6);
    xSemaphoreGive(xMutexI2C);

    if (!ok) {
        return false;
    }

    const int16_t rawX = (int16_t)((raw[1] << 8) | raw[0]);
    const int16_t rawY = (int16_t)((raw[3] << 8) | raw[2]);
    const int16_t rawZ = (int16_t)((raw[5] << 8) | raw[4]);

    ax = rawX * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    ay = rawY * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    az = rawZ * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;

    return true;
}

// =======================================================
// RTC HELPER
// =======================================================
static uint32_t rtc_getUnixTimestamp() {
    uint32_t ts = (uint32_t)(millis() / 1000UL);

    if (!rtcOK) {
        return ts;
    }

    if (xSemaphoreTake(xMutexI2C, pdMS_TO_TICKS(20)) == pdTRUE) {
        ts = rtc.now().unixtime();
        xSemaphoreGive(xMutexI2C);
    }

    return ts;
}

// =======================================================
// CSV / SD HELPERS
// =======================================================
static void writeCsvHeaderIfNeeded() {
    if (!sdOK) return;

    if (xSemaphoreTake(xMutexSD, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (!SD.exists("/wbv_log.csv")) {
            File f = SD.open("/wbv_log.csv", FILE_WRITE);
            if (f) {
                f.println("timestamp,raw_ax,raw_ay,raw_az,awx,awy,awz,av,n_samples");
                f.close();
            }
        }
        xSemaphoreGive(xMutexSD);
    }
}

static void appendWbvCsv(const WbvRmsData_t &d) {
    if (sdOK) {
        if (xSemaphoreTake(xMutexSD, pdMS_TO_TICKS(100)) == pdTRUE) {
            File f = SD.open("/wbv_log.csv", FILE_APPEND);

            if (f) {
                f.printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%lu\n",
                         d.timestamp,
                         d.rawAx, d.rawAy, d.rawAz,
                         d.awx, d.awy, d.awz, d.av,
                         d.n_samples);
                f.close();
            } else {
                LOG_E("SD", "Cannot open /wbv_log.csv");
            }

            xSemaphoreGive(xMutexSD);
        }
    }

    LOG_I("CSV", "%lu,raw=(%.3f,%.3f,%.3f),aw=(%.4f,%.4f,%.4f),av=%.4f,n=%lu",
          d.timestamp,
          d.rawAx, d.rawAy, d.rawAz,
          d.awx, d.awy, d.awz, d.av,
          d.n_samples);
}

// =======================================================
// TASK: WBV ACQUISITION
// =======================================================
static void vTaskWBVAcquisition(void *pvParameters) {
    static const char *TAG = "WBV_ACQ";

    BiquadCascade filterX(coeff_wd, NUM_SECTIONS_WD);
    BiquadCascade filterY(coeff_wd, NUM_SECTIONS_WD);
    BiquadCascade filterZ(coeff_wk, NUM_SECTIONS_WK);

    float sumX2 = 0.0f;
    float sumY2 = 0.0f;
    float sumZ2 = 0.0f;
    uint32_t sampleCount = 0;

    float lastRawX = 0.0f;
    float lastRawY = 0.0f;
    float lastRawZ = 0.0f;

    uint32_t nextSampleUs = micros();

    LOG_I(TAG, "Started on Core %d", xPortGetCoreID());

    while (true) {
        const SystemState_t state = getSystemState();

        if (state != SYS_LOGGING) {
            filterX.reset();
            filterY.reset();
            filterZ.reset();

            sumX2 = 0.0f;
            sumY2 = 0.0f;
            sumZ2 = 0.0f;
            sampleCount = 0;

            vTaskDelay(pdMS_TO_TICKS(20));
            nextSampleUs = micros() + WBV_PERIOD_US;
            continue;
        }

        const uint32_t now = micros();

        if ((int32_t)(now - nextSampleUs) >= 0) {
            nextSampleUs += WBV_PERIOD_US;

            float ax = 0.0f;
            float ay = 0.0f;
            float az = 0.0f;

            if (!adxl345_read(ax, ay, az)) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            lastRawX = ax;
            lastRawY = ay;
            lastRawZ = az;

            const float axWd = filterX.process(ax);
            const float ayWd = filterY.process(ay);
            const float azWk = filterZ.process(az);

            sumX2 += axWd * axWd;
            sumY2 += ayWd * ayWd;
            sumZ2 += azWk * azWk;
            sampleCount++;

            if (sampleCount >= WBV_EPOCH_SAMPLES) {
                WbvRmsData_t result;
                result.rawAx = lastRawX;
                result.rawAy = lastRawY;
                result.rawAz = lastRawZ;

                result.n_samples = sampleCount;
                result.awx = sqrtf(sumX2 / sampleCount);
                result.awy = sqrtf(sumY2 / sampleCount);
                result.awz = sqrtf(sumZ2 / sampleCount);

                result.av = sqrtf(
                    (1.4f * result.awx) * (1.4f * result.awx) +
                    (1.4f * result.awy) * (1.4f * result.awy) +
                    (       result.awz) * (       result.awz)
                );

                result.timestamp = rtc_getUnixTimestamp();

                latestWbvData = result;
                latestWbvValid = true;

                if (xQueueSend(xQueueWBVData, &result, 0) != pdTRUE) {
                    LOG_W(TAG, "Queue full, WBV epoch dropped");
                }

                LOG_I(TAG, "RAW ax=%.3f ay=%.3f az=%.3f | awx=%.4f awy=%.4f awz=%.4f av=%.4f",
                      result.timestamp,
                      result.rawAx, result.rawAy, result.rawAz,
                      result.awx, result.awy, result.awz, result.av);

                sumX2 = 0.0f;
                sumY2 = 0.0f;
                sumZ2 = 0.0f;
                sampleCount = 0;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// =======================================================
// TASK: DATA LOGGER
// =======================================================
static void vTaskDataLogger(void *pvParameters) {
    static const char *TAG = "LOGGER";

    LOG_I(TAG, "Started on Core %d", xPortGetCoreID());

    writeCsvHeaderIfNeeded();

    WbvRmsData_t wbvData;

    while (true) {
        if (xQueueReceive(xQueueWBVData, &wbvData, pdMS_TO_TICKS(LOGGER_PERIOD_MS)) == pdTRUE) {
            appendWbvCsv(wbvData);
        }
    }
}

// =======================================================
// TASK: HMI AND CONTROLLER
// =======================================================
static void vTaskHMIAndController(void *pvParameters) {
    static const char *TAG = "HMI";

    bool buttonLastState = HIGH;
    bool buttonStable = HIGH;
    uint32_t debounceStartMs = 0;
    uint32_t oledTick = 0;

    LOG_I(TAG, "Started on Core %d", xPortGetCoreID());

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(HMI_PERIOD_MS));

        const uint32_t nowMs = millis();
        const bool rawButton = (bool)digitalRead(PIN_BUTTON);

        bool buttonPressedEvent = false;

        if (rawButton != buttonLastState) {
            debounceStartMs = nowMs;
            buttonLastState = rawButton;
        }

        if ((nowMs - debounceStartMs) >= BUTTON_DEBOUNCE_MS) {
            if (rawButton != buttonStable) {
                buttonStable = rawButton;

                if (buttonStable == LOW) {
                    buttonPressedEvent = true;
                }
            }
        }

        if (buttonPressedEvent) {
            const SystemState_t s = getSystemState();

            if (s == SYS_READY) {
                setSystemState(SYS_LOGGING);
                LOG_I(TAG, "FSM: READY -> LOGGING");
            } else if (s == SYS_LOGGING) {
                setSystemState(SYS_READY);
                LOG_I(TAG, "FSM: LOGGING -> READY");
            } else if (s == SYS_ERROR) {
                wbvSensorOK = adxl345_autoDetectAndInit();
                if (wbvSensorOK) {
                    setSystemState(SYS_READY);
                    LOG_I(TAG, "FSM: ERROR -> READY");
                }
            }
        }

        digitalWrite(PIN_LED_STATUS, getSystemState() == SYS_LOGGING ? HIGH : LOW);

        oledTick++;
        if (oledTick >= OLED_UPDATE_DIVIDER) {
            oledTick = 0;

            if (oledOK) {
                if (xSemaphoreTake(xMutexI2C, pdMS_TO_TICKS(50)) == pdTRUE) {
                    oled.clearDisplay();
                    oled.setTextSize(1);
                    oled.setTextColor(SSD1306_WHITE);
                    oled.setCursor(0, 0);

                    oled.println("WBV MAIN UNIT");
                    oled.println("=============");
                    oled.print("State: ");
                    oled.println(stateToString(getSystemState()));
                    oled.print("ADXL: "); oled.println(wbvSensorOK ? "OK" : "ERR");
                    oled.print("RTC : "); oled.println(rtcOK ? "OK" : "ERR");
                    oled.print("SD  : "); oled.println(sdOK ? "OK" : "ERR");

                    if (latestWbvValid) {
                        oled.print("awx: "); oled.println(latestWbvData.awx, 3);
                        oled.print("awy: "); oled.println(latestWbvData.awy, 3);
                        oled.print("awz: "); oled.println(latestWbvData.awz, 3);
                        oled.print("av : "); oled.println(latestWbvData.av, 3);
                    } else {
                        oled.println("No WBV data yet");
                    }

                    oled.display();
                    xSemaphoreGive(xMutexI2C);
                }
            }
        }
    }
}

// =======================================================
// SELF TEST
// =======================================================
static void runSelfTest() {
    LOG_I("INIT", "Running WBV Main Unit self-test");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);

    wbvSensorOK = adxl345_autoDetectAndInit();

    if (wbvSensorOK) {
        LOG_I("INIT", "ADXL345 WBV init OK at 0x%02X", adxlAddress);
    } else {
        LOG_E("INIT", "ADXL345 WBV NOT FOUND");
    }

    if (xSemaphoreTake(xMutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        rtcOK = rtc.begin();
        if (rtcOK && rtc.lostPower()) {
            LOG_W("INIT", "RTC lost power, setting compile time");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        xSemaphoreGive(xMutexI2C);
    }

    LOG_I("INIT", "RTC init %s", rtcOK ? "OK" : "FAIL");

    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SD_CS);
    sdOK = SD.begin(PIN_SD_CS);

    LOG_I("INIT", "SD Card init %s", sdOK ? "OK" : "FAIL");

    if (xSemaphoreTake(xMutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
        if (oledOK) {
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setTextColor(SSD1306_WHITE);
            oled.setCursor(0, 0);
            oled.println("WBV MAIN UNIT");
            oled.println("Booting...");
            oled.display();
        }
        xSemaphoreGive(xMutexI2C);
    }

    LOG_I("INIT", "OLED init %s", oledOK ? "OK" : "FAIL");

    Wire.setClock(400000);

    LOG_I("INIT", "Self-test result: ADXL=%d RTC=%d SD=%d OLED=%d",
          wbvSensorOK, rtcOK, sdOK, oledOK);
}

// =======================================================
// RTOS OBJECTS
// =======================================================
static void createRTOSObjects() {
    xQueueWBVData = xQueueCreate(QUEUE_WBV_LENGTH, sizeof(WbvRmsData_t));
    configASSERT(xQueueWBVData != nullptr);

    xMutexI2C = xSemaphoreCreateMutex();
    xMutexSD = xSemaphoreCreateMutex();
    xMutexState = xSemaphoreCreateMutex();

    configASSERT(xMutexI2C != nullptr);
    configASSERT(xMutexSD != nullptr);
    configASSERT(xMutexState != nullptr);
}

// =======================================================
// TASK CREATION
// =======================================================
static void createTasks() {
    BaseType_t res;

    res = xTaskCreatePinnedToCore(
        vTaskWBVAcquisition,
        "WBV_ACQ",
        TASK_STACK_WBV,
        nullptr,
        TASK_PRIO_WBV,
        &hTaskWBV,
        CORE_DSP
    );
    configASSERT(res == pdPASS);

    res = xTaskCreatePinnedToCore(
        vTaskDataLogger,
        "LOGGER",
        TASK_STACK_LOGGER,
        nullptr,
        TASK_PRIO_LOGGER,
        &hTaskLogger,
        CORE_PERIPHERAL
    );
    configASSERT(res == pdPASS);

    res = xTaskCreatePinnedToCore(
        vTaskHMIAndController,
        "HMI",
        TASK_STACK_HMI,
        nullptr,
        TASK_PRIO_HMI,
        &hTaskHMI,
        CORE_PERIPHERAL
    );
    configASSERT(res == pdPASS);

    LOG_I("INIT", "All tasks created");
}

// =======================================================
// ARDUINO SETUP
// =======================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("==========================================================");
    Serial.println("  WBV Main Unit ESP32 Firmware                            ");
    Serial.println("  ADXL345 + Wd/Wk + RTC + SD + OLED HMI                   ");
    Serial.println("==========================================================");

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);

    systemState = SYS_INIT;

    createRTOSObjects();
    runSelfTest();

    if (wbvSensorOK) {
        setSystemState(SYS_READY);
        LOG_I("INIT", "FSM: INIT -> READY");
    } else {
        setSystemState(SYS_ERROR);
        LOG_E("INIT", "FSM: INIT -> ERROR");
    }

    createTasks();

    for (int i = 0; i < 2; i++) {
        digitalWrite(PIN_LED_STATUS, HIGH);
        delay(100);
        digitalWrite(PIN_LED_STATUS, LOW);
        delay(100);
    }

    LOG_I("INIT", "Setup complete. Press button to start/stop logging.");
}

// =======================================================
// ARDUINO LOOP
// =======================================================
void loop() {
    vTaskDelay(portMAX_DELAY);
}
