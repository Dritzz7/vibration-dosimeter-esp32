#include <Arduino.h>

#define RED_PIN 25   // GPIO25, matches the wiring above
#define GREEN_PIN 26 // GPIO26
#define BLUE_PIN 27  // GPIO27

void setup() {
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
}

void loop() {
    analogWrite(RED_PIN, 255);   // Red at full brightness
    analogWrite(GREEN_PIN, 0);   // Green off
    analogWrite(BLUE_PIN, 0);    // Blue off
    delay(1000);
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 255); // Green at full brightness
    analogWrite(BLUE_PIN, 0);
    delay(1000);
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 255);  // Blue at full brightness
    delay(1000);
}
