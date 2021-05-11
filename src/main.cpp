#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <utility/imumaths.h>

const int IMU_Connections = 1;

Adafruit_BNO055 bno[IMU_Connections] = Adafruit_BNO055(55, 0x28);
BluetoothSerial SerialBT;

void SwitchI2C(uint8_t bus);  //TCA9548A
void send_serial();

float QuatDataArray[IMU_Connections * 4];

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ZAKOTrackerV1.0");
    Serial.println("The device started, now you can pair it with bluetooth!");

    /*for (int i = 0; i < IMU_Connections; i++) {
        if (i > 2) {
            SwitchI2C(i + 3);
        } else {
            SwitchI2C(i);
        }
        if (!bno[i].begin(bno->OPERATION_MODE_IMUPLUS)) {
            Serial.print("BNO055_No.");
            Serial.print(i);
            Serial.print(" not detected\n");
        }
    }*/
    SwitchI2C(2);
    if (!bno[0].begin(bno->OPERATION_MODE_IMUPLUS)) {
        /*Serial.print("BNO055_No.");
        Serial.print("2");
        Serial.print(" not detected\n");*/
    }
}

void loop() {
    /*for (int i = 0; i < IMU_Connections; i++) {
        if (i > 2) {
            SwitchI2C(i + 3);
        } else {
            SwitchI2C(i);
        }
        imu::Quaternion quat = bno[i].getQuat();
        QuatDataArray[i * 4] = quat.x();
        QuatDataArray[i * 4 + 1] = quat.y();
        QuatDataArray[i * 4 + 2] = quat.z();
        QuatDataArray[i * 4 + 3] = quat.w();
    }*/
    SwitchI2C(2);
    imu::Quaternion quat = bno[0].getQuat();
    QuatDataArray[0] = quat.x();
    QuatDataArray[1] = quat.y();
    QuatDataArray[2] = quat.z();
    QuatDataArray[3] = quat.w();
    send_serial();
}

void SwitchI2C(uint8_t bus) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << bus);
    Wire.endTransmission();
}

void send_serial() {
    SerialBT.print("s\t");
    for (int i = 0; i < IMU_Connections * 4 - 1; i++) {
        SerialBT.print(QuatDataArray[i]);
        SerialBT.print("\t");
    }
    SerialBT.print(QuatDataArray[IMU_Connections * 4 - 1]);
    SerialBT.print("\n");
}