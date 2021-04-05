#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <utility/imumaths.h>

#include <SparkFun_I2C_Mux_Arduino_Library.h>

Adafruit_BNO055 bno[4] = Adafruit_BNO055(55, 0x70);

BluetoothSerial SerialBT;

void SwitchI2C(uint8_t bus);  //TCA9548A
void send_serial(float inDataArray[20]);

float QuatDataArray[20];

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ZAKOTrackerV1.0");
    Serial.println("The device started, now you can pair it with bluetooth!");

    for (int i = 0; i < 5; i++) {
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
    for (int i = 0; i < 5; i++) {
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

void send_serial(float inDataArray[20]) {
    SerialBT.print("s\t");
    for (int i = 0; i < 20; i++) {
        SerialBT.print(QuatDataArray[i]);
        SerialBT.print("\t");
    }
    SerialBT.print("\n");
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