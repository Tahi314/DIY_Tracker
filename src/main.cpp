#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
    Serial.begin(115200);

    if (!bno.begin()) {
        Serial.print("No BNO055 detected");
        while (1)
            ;
    }
}

void loop() {}