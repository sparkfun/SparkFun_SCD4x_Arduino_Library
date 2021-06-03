/*
  Reading CO2, humidity and temperature from the SCD4x
  By: Paul Clark
  Based on earlier code by: Nathan Seidle
  SparkFun Electronics
  Date: June 3rd, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/nnnnn

  This example prints the current CO2 level, relative humidity, and temperature in C.

  Hardware Connections:
  Attach RedBoard to computer using a USB cable.
  Connect SCD40/41 to RedBoard using Qwiic cable.
  Open Serial Monitor at 115200 baud.
*/

#include <Wire.h>

#include "SparkFun_SCD4x_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD4x
SCD4x mySensor;

void setup()
{
  Serial.begin(115200);
  Serial.println("SCD4x Example");
  Wire.begin();

  mySensor.enableDebugging(); // Uncomment this line to get helpful debug messages on Serial

  //.begin will start periodic measurements for us (see the later examples for details on how to overrride this)
  if (mySensor.begin() == false)
  {
    Serial.println("Sensor not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }

  //The SCD4x has data ready every five seconds
}

void loop()
{
  if (mySensor.getDataReadyStatus()) // getDataReadyStatus will return true when fresh data is available
  {
    Serial.println();

    Serial.print("CO2(ppm):");
    Serial.print(mySensor.getCO2());

    Serial.print("\tTemperature(C):");
    Serial.print(mySensor.getTemperature(), 1);

    Serial.print("\tHumidity(%RH):");
    Serial.print(mySensor.getHumidity(), 1);

    Serial.println();
  }
  else
    Serial.print(".");

  delay(500);
}
