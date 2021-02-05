#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
    Serial.begin(115200);

    if (!bno.begin(bno.OPERATION_MODE_NDOF)) {
        Serial.print("No BNO055 detected");
        while (1)
            ;
    }
}

void loop() {
    read_sensor();
    delay(BNO055_SAMPLERATE_DELAY_MS);
}

void read_sensor() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
}

void send_serial(float x, float y, float z) {
    if (Serial.available() == 1) {
        byte inBuf[1];
        Serial.readBytes(inBuf, 1);
        if (inBuf[0] == 's') {
            byte outBuf[24];
            outBuf[0] = 's';
            outBuf[1] = (int16_t)(x * 100) >> 8;
            outBuf[2] = (int16_t)(x * 100) & 0xFF;
            outBuf[3] = (int16_t)(y * 100) >> 8;
            outBuf[4] = (int16_t)(y * 100) & 0xFF;
            outBuf[5] = (int16_t)(z * 100) >> 8;
            outBuf[6] = (int16_t)(z * 100) & 0xFF;
            outBuf[7] = (int16_t)(0) >> 8;
            outBuf[8] = (int16_t)(0) & 0xFF;
            outBuf[9] = (int16_t)(0) >> 8;
            outBuf[10] = (int16_t)(0) & 0xFF;
            outBuf[11] = (int16_t)(0) >> 8;
            outBuf[12] = (int16_t)(0) & 0xFF;
            outBuf[13] = (int16_t)(0) >> 8;
            outBuf[14] = (int16_t)(0) & 0xFF;
            outBuf[15] = (int16_t)(0) >> 8;
            outBuf[16] = (int16_t)(0) & 0xFF;
            outBuf[17] = (int16_t)(0) >> 8;
            outBuf[18] = (int16_t)(0) & 0xFF;
            outBuf[19] = (int16_t)(0) >> 8;
            outBuf[20] = (int16_t)(0) & 0xFF;
            outBuf[21] = (int16_t)(0) >> 8;
            outBuf[22] = (int16_t)(0) & 0xFF;
            outBuf[23] = 'e';
            Serial.write(outBuf, 24);
        } else {
            while (Serial.available() > 0) Serial.read();
        }
    }
    if (Serial.available() > 1) {
        while (Serial.available() > 0) Serial.read();
    }
}