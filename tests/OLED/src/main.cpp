#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SH1106 display(OLED_RESET);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);  // Change to your board's SDA/SCL pins if needed

  display.begin(SH1106_SWITCHCAPVCC, 0x3C);
  delay(100);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("OLED test");
  display.println("ESP32 + SH1106");
  display.drawRect(10, 20, 100, 30, WHITE);
  display.fillRect(20, 28, 80, 14, WHITE);
  display.display();
}

void loop() {
  delay(1000);
}