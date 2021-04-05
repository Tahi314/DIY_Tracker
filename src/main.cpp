#include <Wire.h>

#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
QWIICMUX myMux;

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic Mux Shield Read Example");

  Wire1.begin(); //This line will fail to compile on an Uno. Use a dev platform with multiple Wire ports

  //Setup mux to use Wire1. If you have multiple muxes, pass address as first argument
  //ADR0 must have a the jumper closed with solder to use address 0x71. Use 0x70 for all jumpers open.
  if (myMux.begin(0x70, Wire1) == false)
  {
    Serial.println("Mux not detected. Freezing...");
    while (1)
      ;
  }
  Serial.println("Mux detected");

  myMux.setPort(1); //Connect master to port labeled '1' on the mux

  byte currentPortNumber = myMux.getPort();
  Serial.print("CurrentPort: ");
  Serial.println(currentPortNumber);

  Serial.println("Begin scanning for I2C devices");
}

void loop()
{
  Serial.println();

  byte nDevices = 0;
  for (byte address = 1; address < 127; address++)
  {
    Wire1.beginTransmission(address);
    byte error = Wire1.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 0x10)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();

      nDevices++;
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("Done");

  delay(1000);
}