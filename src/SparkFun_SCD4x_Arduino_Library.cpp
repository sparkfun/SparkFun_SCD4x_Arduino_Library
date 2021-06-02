/*
  This is a library written for the SCD4x family of CO2 sensors
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/nnnnn

  Written by Paul Clark @ SparkFun Electronics, June 2nd, 2021

  The SCD41 measures CO2 from 400ppm to 5000ppm with an accuracy of +/- 40ppm + 5% of reading

  This library handles the initialization of the SCD4x and outputs
  CO2 levels, relative humidty, and temperature.

  https://github.com/sparkfun/SparkFun_SCD4x_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.13

  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for more details.
*/

#include "SparkFun_SCD4x_Arduino_Library.h"

SCD4x::SCD4x(scd4x_sensor_type_e sensorType)
{
  // Constructor
  _sensorType = sensorType;
}

//Initialize the Serial port
#ifdef USE_TEENSY3_I2C_LIB
bool SCD30::begin(i2c_t3 &wirePort, bool autoCalibrate, bool measBegin)
#else
bool SCD30::begin(TwoWire &wirePort, bool autoCalibrate, bool measBegin)
#endif
{
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  /* Especially during obtaining the ACK BIT after a byte sent the SCD30 is using clock stretching  (but NOT only there)!
   * The need for clock stretching is described in the Sensirion_CO2_Sensors_SCD30_Interface_Description.pdf
   *
   * The default clock stretch (maximum wait time) on the ESP8266-library (2.4.2) is 230us which is set during _i2cPort->begin();
   * In the current implementation of the ESP8266 I2C driver there is NO error message when this time expired, while
   * the clock stretch is still happening, causing uncontrolled behaviour of the hardware combination.
   *
   * To set ClockStretchlimit() a check for ESP8266 boards has been added in the driver.
   *
   * With setting to 20000, we set a max timeout of 20mS (> 20x the maximum measured) basically disabling the time-out
   * and now wait for clock stretch to be controlled by the client.
   */

#if defined(ARDUINO_ARCH_ESP8266)
  _i2cPort->setClockStretchLimit(200000);
#endif

  uint16_t fwVer;
  if (getFirmwareVersion(&fwVer) == false) // Read the firmware version. Return false if the CRC check fails.
    return (false);

  if (_printDebug == true)
  {
    _debugPort->print(F("SCD30 begin: got firmware version 0x"));
    _debugPort->println(fwVer, HEX);
  }

  if (measBegin == false) // Exit now if measBegin is false
    return (true);

  //Check for device to respond correctly
  if (beginMeasuring() == true) //Start continuous measurements
  {
    setMeasurementInterval(2);             //2 seconds between measurements
    setAutoSelfCalibration(autoCalibrate); //Enable auto-self-calibration

    return (true);
  }

  return (false); //Something went wrong
}

//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
void SCD30::enableDebugging(Stream &debugPort)
{
  _debugPort = &debugPort;
  _printDebug = true;
}

//Start periodic measurements. See 3.5.1
bool SCD4x::startPeriodicMeasurement(void)
{
  return sendCommand(SCD4x_COMMAND_START_PERIODIC_MEASUREMENT);
}

//Stop periodic measurements. See 3.5.3
//Note that the sensor will only respond to other commands after waiting 500 ms
//after issuing the stop_periodic_measurement command.
bool SCD4x::stopPeriodicMeasurement(void)
{
  bool success = sendCommand(SCD4x_COMMAND_STOP_PERIODIC_MEASUREMENT);
  delay(500);
  return (success);
}

//Get 9 bytes from SCD4x. See 3.5.2
//Updates global variables with floats
//Returns true if success
bool SCD4x::readMeasurement(void)
{
  //Verify we have data from the sensor
  if (dataAvailable() == false)
    return (false);

  scd4x_unsigned16Bytes_t tempCO2;
  tempCO2.unsigned16 = 0;
  scd4x_unsigned16Bytes_t  tempHumidity;
  tempHumidity.unsigned16 = 0;
  scd4x_unsigned16Bytes_t  tempTemperature;
  tempTemperature.unsigned16 = 0;

  _i2cPort->beginTransmission(SCD4x_ADDRESS);
  _i2cPort->write(SCD4x_COMMAND_READ_MEASUREMENT >> 8);   //MSB
  _i2cPort->write(SCD4x_COMMAND_READ_MEASUREMENT & 0xFF); //LSB
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK

  delay(1);

  uint8_t receivedBytes = (uint8_t)_i2cPort->requestFrom((uint8_t)SCD4x_ADDRESS, (uint8_t)9);
  bool error = false;
  if (_i2cPort->available())
  {
    byte bytesToCrc[2];
    for (byte x = 0; x < 9; x++)
    {
      byte incoming = _i2cPort->read();

      switch (x)
      {
      case 0:
      case 1:
        tempCO2.bytes[x == 0 ? 1 : 0] = incoming;
        bytesToCrc[x] = incoming;
        break;
      case 3:
      case 4:
        tempTemperature.bytes[x == 3 ? 1 : 0] = incoming;
        bytesToCrc[x % 2] = incoming;
        break;
      case 6:
      case 7:
        tempHumidity.bytes[x == 6 ? 1 : 0] = incoming;
        bytesToCrc[x % 2] = incoming;
        break;
      default:
        //Validate CRC
        uint8_t foundCrc = computeCRC8(bytesToCrc, 2);
        if (foundCrc != incoming)
        {
          if (_printDebug == true)
          {
            _debugPort->print(F("SCD4x::readMeasurement: found CRC in byte "));
            _debugPort->print(x);
            _debugPort->print(F(", expected 0x"));
            _debugPort->print(foundCrc, HEX);
            _debugPort->print(F(", got 0x"));
            _debugPort->println(incoming, HEX);
          }
          error = true;
        }
        break;
      }
    }
  }
  else
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("SCD4x::readMeasurement: no SCD4x data found from I2C, I2C claims we should receive "));
      _debugPort->print(receivedBytes);
      _debugPort->println(F(" bytes"));
    }
    return (false);
  }

  if (error)
  {
    if (_printDebug == true)
      _debugPort->println(F("SCD4x::readMeasurement: encountered error reading SCD4x data."));
    return (false);
  }
  //Now copy the int16s into their associated floats
  co2 = (float)tempCO2.unsigned16;
  temperature = -45 + (((float)tempTemperature.unsigned16) * 175 / 65536);
  humidity = ((float)tempHumidity.unsigned16) * 100 / 65536;

  //Mark our global variables as fresh
  co2HasBeenReported = false;
  humidityHasBeenReported = false;
  temperatureHasBeenReported = false;

  return (true); //Success! New data available in globals.
}

//Returns true when data is available
bool SCD4x::dataAvailable(void)
{
  uint16_t response;
  bool success = readRegister(SCD4x_COMMAND_GET_DATA_READY_STATUS, &response, 1);

  if (success == false)
    return (false);

  //If the least significant 11 bits of word[0] are 0 → data not ready
  //else → data ready for read-out
  if ((response & 0x07ff) == 0x0000)
    return (false);
  return (true);
}

//Returns the latest available CO2 level
//If the current level has already been reported, trigger a new read
uint16_t SCD30::getCO2(void)
{
  if (co2HasBeenReported == true) //Trigger a new read
    readMeasurement();            //Pull in new co2, humidity, and temp into global vars

  co2HasBeenReported = true;

  return (uint16_t)co2; //Cut off decimal as co2 is 0 to 10,000
}

//Returns the latest available humidity
//If the current level has already been reported, trigger a new read
float SCD30::getHumidity(void)
{
  if (humidityHasBeenReported == true) //Trigger a new read
    readMeasurement();                 //Pull in new co2, humidity, and temp into global vars

  humidityHasBeenReported = true;

  return humidity;
}

//Returns the latest available temperature
//If the current level has already been reported, trigger a new read
float SCD30::getTemperature(void)
{
  if (temperatureHasBeenReported == true) //Trigger a new read
    readMeasurement();                    //Pull in new co2, humidity, and temp into global vars

  temperatureHasBeenReported = true;

  return temperature;
}

//Enables or disables the ASC
bool SCD30::setAutoSelfCalibration(bool enable)
{
  if (enable)
    return sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION, 1); //Activate continuous ASC
  else
    return sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION, 0); //Deactivate continuous ASC
}

//Set the forced recalibration factor. See 1.3.7.
//The reference CO2 concentration has to be within the range 400 ppm ≤ cref(CO2) ≤ 2000 ppm.
bool SCD30::setForcedRecalibrationFactor(uint16_t concentration)
{
  if (concentration < 400 || concentration > 2000)
  {
    return false; //Error check.
  }
  return sendCommand(COMMAND_SET_FORCED_RECALIBRATION_FACTOR, concentration);
}

//Set the temperature offset. See 3.6.1
bool SCD4x::setTemperatureOffset(float tempOffset)
{
  scd4x_signedUnsigned16_t  signedUnsigned; // Avoid any ambiguity casting int16_t to uint16_t
  signedUnsigned.signed16 = tempOffset * 65536 / 175; // Toffset [°C] * 2^16 / 175
  return sendCommand(SCD4x_COMMAND_SET_TEMPERATURE_OFFSET, signedUnsigned.unsigned16);
}

//Get the temperature offset. See 3.6.2
float SCD4x::getTemperatureOffset(void)
{
  float offset;
  bool success = getTemperatureOffset(&offset);
  if ((success == false) && (_printDebug == true))
  {
    _debugPort->println(F("SCD4x::getTemperatureOffset: failed to read offset. Returning 0.0"));
  }
  return (offset);
}

//Get the temperature offset. See 3.6.2
bool SCD4x::getTemperatureOffset(float *offset)
{
  scd4x_signedUnsigned16_t signedUnsigned; // Avoid any ambiguity casting int16_t to uint16_t
  signedUnsigned.unsigned16 = 0; // Return zero if readRegister fails
  bool success = readRegister(SCD4x_COMMAND_GET_TEMPERATURE_OFFSET, &signedUnsigned.unsigned16, 1);
  *offset = ((float)signedUnsigned.signed16) * 175.0 / 65535.0;
  return (success);
}

//Get the altitude compenstation. See 1.3.9.
uint16_t SCD30::getAltitudeCompensation(void)
{
  return readRegister(COMMAND_SET_ALTITUDE_COMPENSATION);
}

//Set the altitude compenstation. See 1.3.9.
bool SCD30::setAltitudeCompensation(uint16_t altitude)
{
  return sendCommand(COMMAND_SET_ALTITUDE_COMPENSATION, altitude);
}

//Set the pressure compenstation. This is passed during measurement startup.
//mbar can be 700 to 1200
bool SCD30::setAmbientPressure(uint16_t pressure_mbar)
{
  if (pressure_mbar < 700 || pressure_mbar > 1200)
  {
    return false;
  }
  return sendCommand(COMMAND_CONTINUOUS_MEASUREMENT, pressure_mbar);
}

// SCD30 soft reset
void SCD30::reset()
{
  sendCommand(COMMAND_RESET);
}

// Get the current ASC setting
bool SCD30::getAutoSelfCalibration()
{
  uint16_t response = readRegister(COMMAND_AUTOMATIC_SELF_CALIBRATION);
  if (response == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//Begins continuous measurements
//Continuous measurement status is saved in non-volatile memory. When the sensor
//is powered down while continuous measurement mode is active SCD30 will measure
//continuously after repowering without sending the measurement command.
//Returns true if successful
bool SCD30::beginMeasuring(uint16_t pressureOffset)
{
  return (sendCommand(COMMAND_CONTINUOUS_MEASUREMENT, pressureOffset));
}

//Overload - no pressureOffset
bool SCD30::beginMeasuring(void)
{
  return (beginMeasuring(0));
}

// Stop continuous measurement
bool SCD30::StopMeasurement(void)
{
  return (sendCommand(COMMAND_STOP_MEAS));
}

//Sets interval between measurements
//2 seconds to 1800 seconds (30 minutes)
bool SCD30::setMeasurementInterval(uint16_t interval)
{
  return sendCommand(COMMAND_SET_MEASUREMENT_INTERVAL, interval);
}

//Sends a command along with arguments and CRC
bool SCD4x::sendCommand(uint16_t command, uint16_t arguments)
{
  uint8_t data[2];
  data[0] = arguments >> 8;
  data[1] = arguments & 0xFF;
  uint8_t crc = computeCRC8(data, 2); //Calc CRC on the arguments only, not the command

  _i2cPort->beginTransmission(SCD4x_ADDRESS);
  _i2cPort->write(command >> 8);     //MSB
  _i2cPort->write(command & 0xFF);   //LSB
  _i2cPort->write(arguments >> 8);   //MSB
  _i2cPort->write(arguments & 0xFF); //LSB
  _i2cPort->write(crc);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK

  return (true);
}

//Sends just a command, no arguments, no CRC
bool SCD4x::sendCommand(uint16_t command)
{
  _i2cPort->beginTransmission(SCD30_ADDRESS);
  _i2cPort->write(command >> 8);   //MSB
  _i2cPort->write(command & 0xFF); //LSB
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK

  return (true);
}

//Gets two bytes from SCD4x plus CRC.
//Returns true if endTransmission returns zero _and_ the CRC check is valid
bool SCD4x::readRegister(uint16_t registerAddress, uint16_t *response, uint16_t delayMillis)
{
  _i2cPort->beginTransmission(SCD4x_ADDRESS);
  _i2cPort->write(registerAddress >> 8);   //MSB
  _i2cPort->write(registerAddress & 0xFF); //LSB
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK

  delay(delayMillis);

  _i2cPort->requestFrom((uint8_t)SCD4x_ADDRESS, (uint8_t)3); // Request data and CRC
  if (_i2cPort->available())
  {
    uint8_t data[2];
    data[0] = _i2cPort->read();
    data[1] = _i2cPort->read();
    uint8_t crc = _i2cPort->read();
    *response = (uint16_t)data[0] << 8 | data[1];
    uint8_t expectedCRC = computeCRC8(data, 2);
    if (crc == expectedCRC) // Return true if CRC check is OK
      return (true);
    if (_printDebug == true)
    {
      _debugPort->print(F("SCD4x::readRegister: CRC fail: expected 0x"));
      _debugPort->print(expectedCRC, HEX);
      _debugPort->print(F(", got 0x"));
      _debugPort->println(crc, HEX);
    }
  }
  return (false);
}

//Given an array and a number of bytes, this calculate CRC8 for those bytes
//CRC is only calc'd on the data portion (two bytes) of the four bytes being sent
//From: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
//Tested with: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
//x^8+x^5+x^4+1 = 0x31
uint8_t SCD4x::computeCRC8(uint8_t data[], uint8_t len)
{
  uint8_t crc = 0xFF; //Init with 0xFF

  for (uint8_t x = 0; x < len; x++)
  {
    crc ^= data[x]; // XOR-in the next input byte

    for (uint8_t i = 0; i < 8; i++)
    {
      if ((crc & 0x80) != 0)
        crc = (uint8_t)((crc << 1) ^ 0x31);
      else
        crc <<= 1;
    }
  }

  return crc; //No output reflection
}
