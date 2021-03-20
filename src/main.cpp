#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno[4] = Adafruit_BNO055(55, 0x70);

BluetoothSerial SerialBT;

void TCA9548A(uint8_t bus);
void read_sensor(uint8_t num);
void send_serial(float qx, float qy, float qz, float qw);

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ZAKOTrackerV1.0");
    Serial.println("The device started, now you can pair it with bluetooth!");

    for (int i = 0; i < 5; i++) {
        if (!bno[i].begin(bno->OPERATION_MODE_IMUPLUS)) {
            Serial.print("BNO055_");
            Serial.print(i);
            Serial.print(" not detected\n");
            while (1)
                ;
        }
    }
}

void loop() {
    for (int i = 0; i < 5; i++) {
        TCA9548A(i);
        read_sensor(i);
    }
}

void TCA9548A(uint8_t bus) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << bus);
    Wire.endTransmission();
}

void read_sensor(uint8_t num) {
    imu::Quaternion quat = bno[num].getQuat();
    send_serial(quat.x(), quat.y(), quat.z(), quat.w());
}

void send_serial(float qx, float qy, float qz, float qw) {
    Serial.print("s\t");
    Serial.print(qx);
    Serial.print("\t");
    Serial.print(qy);
    Serial.print("\t");
    Serial.print(qz);
    Serial.print("\t");
    Serial.println(qw);
}

/*void send_serial(float qx, float qy, float qz, float qw) {
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
}*/