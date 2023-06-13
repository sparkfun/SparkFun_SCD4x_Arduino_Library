/*
  SCD4x Sensor Type Test
  By: Alex Brudner
  Based on earlier code by: Nathan Seidle and Paul Clark
  SparkFun Electronics
  Date: 
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun CO2 Humidity and Temperature Sensor - SCD40:
    - https://www.sparkfun.com/products/22395
  SparkFun CO2 Humidity and Temperature Sensor - SCD41:
    - https://www.sparkfun.com/products/22396

  This example reads the serial number from the device, determines the device type, then prints the device type to serial before spinning.

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
  Serial.println(F("Sensor Type test"));
  Wire.begin();

  // mySensor.enableDebugging(Serial); // Uncomment this line to get helpful debug messages on Serial

  if (mySensor.begin(false, true, false, true) == false) // Do not start periodic measurements
  //measBegin_________/     |     |      |
  //autoCalibrate__________/      |      |
  //skipStopPeriodicMeasurements_/      /
  //pollAndSetDeviceType______________/
  {
    Serial.println(F("Sensor not detected. Please check wiring. Freezing..."));
    while (1)
      ;
  }

  scd4x_sensor_type_e sensor;
  bool success = mySensor.getFeatureSetVersion(&sensor);
  Serial.print(F("Sensor determined to by of type: SCD4"));
  Serial.println(sensor);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Entering loop, spinning...");
  while(1);
}
