#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);  // Pin I2C SDA=21, SCL=22

  if (!display.begin(0x3C, true)) {
    Serial.println("[ERROR] SH1106 OLED tidak terdeteksi pada alamat 0x3C!");
    while (1) {
      delay(100);
    }
  }
  Serial.println("[OK] SH1106 OLED terinisialisasi.");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("OLED test");
  display.println("ESP32 + SH1106");
  display.drawRect(10, 20, 100, 30, SH110X_WHITE);
  display.fillRect(20, 28, 80, 14, SH110X_WHITE);
  display.display();
}

void loop() {
  delay(1000);
}