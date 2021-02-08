#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void read_sensor();
void send_serial(float qx, float qy, float qz, float qw);

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
    // delay(BNO055_SAMPLERATE_DELAY_MS);
}

void read_sensor() {
    // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat = bno.getQuat();
    send_serial(quat.x(), quat.y(), quat.z(), quat.w());
}

void send_serial(float qx, float qy, float qz, float qw) {
    if (Serial.available() == 1) {
        byte inBuf[1];
        Serial.readBytes(inBuf, 1);
        if (inBuf[0] == 's') {
            byte outBuf[10];
            outBuf[0] = 's';
            outBuf[1] = (int16_t)(qx * 100) >> 8;
            outBuf[2] = (int16_t)(qx * 100) & 0xFF;
            outBuf[3] = (int16_t)(qy * 100) >> 8;
            outBuf[4] = (int16_t)(qy * 100) & 0xFF;
            outBuf[5] = (int16_t)(qz * 100) >> 8;
            outBuf[6] = (int16_t)(qz * 100) & 0xFF;
            outBuf[7] = (int16_t)(qw * 100) >> 8;
            outBuf[8] = (int16_t)(qw * 100) & 0xFF;
            outBuf[9] = 'e';
            Serial.write(outBuf, 10);
        } else {
            while (Serial.available() > 0) Serial.read();
        }
    }
    if (Serial.available() > 1) {
        while (Serial.available() > 0) Serial.read();
    }
}