#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno[4] = Adafruit_BNO055(55, 0x28);

BluetoothSerial SerialBT;

void SwitchI2C(uint8_t bus);  //TCA9548A
void send_serial(float inDataArray[8]);

float QuatDataArray[8];

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ZAKOTrackerV1.0");
    Serial.println("The device started, now you can pair it with bluetooth!");

    for (int i = 0; i < 2; i++) {
			SwitchI2C(i);
        if (!bno[i].begin(bno->OPERATION_MODE_IMUPLUS)) {
            Serial.print("BNO055_No.");
            Serial.print(i);
            Serial.print(" not detected\n");
            while (1)
                ;
        }
    }
}

void loop() {
    for (int i = 0; i < 2; i++) {
        SwitchI2C(i);
        imu::Quaternion quat = bno[i].getQuat();
        QuatDataArray[i * 4] = quat.x();
        QuatDataArray[i * 4 + 1] = quat.y();
        QuatDataArray[i * 4 + 2] = quat.z();
        QuatDataArray[i * 4 + 3] = quat.w();
    }
    send_serial(QuatDataArray);
}

void SwitchI2C(uint8_t bus) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << bus);
    Wire.endTransmission();
}

void send_serial(float inDataArray[8]) {
    SerialBT.print("s\t");
    for (int i = 0; i < 8; i++) {
        SerialBT.print(QuatDataArray[i]);
        SerialBT.print("\t");
    }
    SerialBT.print("\n");
}