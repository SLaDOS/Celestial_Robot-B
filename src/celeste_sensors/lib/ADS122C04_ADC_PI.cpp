/**
   \file ADS122C04_ADC_PI.cpp
   \brief Definition of the SFE_ADS122C04 class for interacting with the
          ADS112C04 hardware.

  This file is based on a library written for the TI ADS122C04
  24-Bit 4-Channel 2-kSPS Delta-Sigma ADC With I2C Interface.
  The implementation (ADS122C04_ADC_PI.cpp) has been modified for
  the 16-bit cousin of the ADS122C04 (namely, the ADS112C04).

  If you need to understand this code, then I recommed going through the
  datasheet linked below in parallel.

  Original library written by: Paul Clark (PaulZC)
  Date: May 4th 2020

  Modified by: Robert Mitchell (refmitchell)
  Date: 15/02/2022

  Reason for modification:
  Native operation on RPi using Linux i2c headers (as opposed to Arduino exclusive
  Wire.h) and adaptation for the 16-bit ADS112C04 platform. Modifications to this
  header are minimal.

  Based on: https://github.com/sparkfun/SparkFun_ADS122C04_ADC_Arduino_Library
  (written for the 24-bit ADS122C04 for use with an Arduino).

  Also see the TI datasheet:
  https://www.ti.com/product/ADS112C04
  https://www.ti.com/lit/ds/symlink/ads112c04.pdf

  Software made available under the MIT license (included in the source code).
*/

/*
  The MIT License (MIT)

  Copyright (c) 2020 SparkFun Electronics

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "celeste_sensors/ADS122C04_ADC_PI.hpp"

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
// #include <linux/ioctl.h>
#include <fcntl.h> // open() and O_RDWR
#include <unistd.h> // close()
}

#include <iostream>
#include <sstream>

#define LOG(x) std::cout << x << std::endl;
#include <chrono> // Time durations
#include <thread> // Sleep
#include <cmath>  // Maths utilities

/** Constructor */
SFE_ADS122C04::SFE_ADS122C04(void)
{
  // Constructor
}

//
//Return true if successful

/**
    Attempt communication with the device and initialise it.

    \param deviceAddress The I2C address of the target device.
    \param bus The string identifier of the I2C bus in use (see Linux kernel documentation)
    \return true if success, false otherwise

*/
bool SFE_ADS122C04::begin(uint8_t deviceAddress, std::string bus)
{
  _deviceAddress = deviceAddress; //If provided, store the I2C address from user
  _i2c_fd = 0;
  _wireMode = ADS122C04_RAW_MODE; //Default to using 'safe' settings (disable the IDAC current sources)

  // wait for power-on reset to complete (datasheet says we should do this)
  // minimum of 500us required
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if ((_i2c_fd = open(bus.c_str(), O_RDWR)) < 0) std::exit(1);

  if (ioctl(_i2c_fd, I2C_SLAVE, _deviceAddress) < 0){
      close(_i2c_fd);
      std::exit(1);
  }

  // Get functionality flags
  unsigned long funcs;
  if (ioctl(_i2c_fd, I2C_FUNCS, &funcs) < 0){
    LOG("Failed to read I2C functionality.");
    exit(1);
  }

  reset(); // reset the ADS122C04 (datasheet says we should do this)

  // Default to using 'safe' settings (disable the IDAC current sources)
  return(configureADCmode(ADS122C04_RAW_MODE));
}

/**
   Configure the chip for the selected wire mode. Valid wire modes are defined
   in ADS_122C04_PI.hpp. For the purposes of polarisation sensor operation,
   a new dedicated wire mode (ADS122C04_POL_OP_MODE) is included.

   \param wire_mode The wire mode setting (see definitions and datasheet).
   \param rate The data rate to be set on initialisation.
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::configureADCmode(uint8_t wire_mode, uint8_t rate)
{
  ADS122C04_initParam initParams; // Storage for the chip parameters

  if (wire_mode == ADS122C04_4WIRE_MODE) // 4-wire mode
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_8; // Set the gain to 8
    initParams.pgaBypass = ADS122C04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_1000_UA; // Set the IDAC current to 1mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3; // Route IDAC1 to AIN3
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_4WIRE_MODE; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_4WIRE_HI_TEMP) // 4-wire mode for high temperatures (gain = 4)
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_4; // Set the gain to 4
    initParams.pgaBypass = ADS122C04_PGA_ENABLED; // Enable the PGA
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_1000_UA; // Set the IDAC current to 1mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3; // Route IDAC1 to AIN3
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_4WIRE_HI_TEMP; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_3WIRE_MODE) // 3-wire mode
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_8; // Set the gain to 8
    initParams.pgaBypass = ADS122C04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_500_UA; // Set the IDAC current to 0.5mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN2; // Route IDAC1 to AIN2
    initParams.routeIDAC2 = ADS122C04_IDAC2_AIN3; // Route IDAC2 to AIN3
    _wireMode = ADS122C04_3WIRE_MODE; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_3WIRE_HI_TEMP) // 3-wire mode for high temperatures (gain = 4)
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_4; // Set the gain to 4
    initParams.pgaBypass = ADS122C04_PGA_ENABLED; // Enable the PGA
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_500_UA; // Set the IDAC current to 0.5mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN2; // Route IDAC1 to AIN2
    initParams.routeIDAC2 = ADS122C04_IDAC2_AIN3; // Route IDAC2 to AIN3
    _wireMode = ADS122C04_3WIRE_HI_TEMP; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_2WIRE_MODE) // 2-wire mode
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_8; // Set the gain to 8
    initParams.pgaBypass = ADS122C04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_1000_UA; // Set the IDAC current to 1mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3; // Route IDAC1 to AIN3
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_2WIRE_MODE; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_2WIRE_HI_TEMP) // 2-wire mode for high temperatures (gain = 4)
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_4; // Set the gain to 4
    initParams.pgaBypass = ADS122C04_PGA_ENABLED; // Enable the PGA
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS; // Use the external REF pins
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_1000_UA; // Set the IDAC current to 1mA
    initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3; // Route IDAC1 to AIN3
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_2WIRE_HI_TEMP; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_TEMPERATURE_MODE) // Internal temperature mode
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_1; // Set the gain to 1
    initParams.pgaBypass = ADS122C04_PGA_DISABLED;
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_INTERNAL; // Use the internal 2.048V reference
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_ON; // Enable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF; // Disable the IDAC current
    initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED; // Disable IDAC1
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_TEMPERATURE_MODE; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_RAW_MODE) // Raw mode : disable the IDAC and use the internal reference
  {
    initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
    initParams.gainLevel = ADS122C04_GAIN_1; // Set the gain to 1
    initParams.pgaBypass = ADS122C04_PGA_DISABLED;
    initParams.dataRate = rate; // Set the data rate (samples per second). Defaults to 20
    initParams.opMode = ADS122C04_OP_MODE_NORMAL; // Disable turbo mode
    initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
    initParams.selectVref = ADS122C04_VREF_INTERNAL; // Use the internal 2.048V reference
    initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF; // Disable the temperature sensor
    initParams.dataCounterEn = ADS122C04_DCNT_DISABLE; // Disable the data counter
    initParams.dataCRCen = ADS122C04_CRC_DISABLED; // Disable CRC checking
    initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
    initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF; // Disable the IDAC current
    initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED; // Disable IDAC1
    initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED; // Disable IDAC2
    _wireMode = ADS122C04_RAW_MODE; // Update the wire mode
  }
  else if (wire_mode == ADS122C04_POL_OP_MODE)
    {
      initParams.inputMux = ADS122C04_MUX_AIN1_AIN0;
      initParams.gainLevel = ADS122C04_GAIN_1; // May change step size
      //initParams.gainLevel = ADS122C04_GAIN_8; // May change step size
      initParams.pgaBypass = ADS122C04_PGA_ENABLED; // PGA should be enabled.
      initParams.dataRate = rate;
      initParams.opMode = ADS122C04_OP_MODE_NORMAL;
      initParams.convMode = ADS122C04_CONVERSION_MODE_CONTINUOUS; // May change to continuous (DCNT)
      //initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // May change to continuous (DCNT)
      initParams.selectVref = ADS122C04_VREF_INTERNAL; // Might be an issue
      initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;
      initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;
      initParams.dataCRCen = ADS122C04_CRC_DISABLED;
      initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;
      initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF;
      initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED;
      initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED;
      _wireMode = ADS122C04_POL_OP_MODE;
    }
  else
  {
    if (_printDebug == true)
    {
      LOG("configureADCmode: unknown mode");
    }
    return(false);
  }
  lastInitParams = initParams; // Record last init params
  return(ADS122C04_init(&initParams)); // Configure the chip
}

/**
   Enable the printing of debug messages
*/
void SFE_ADS122C04::enableDebugging(void)
{
  _printDebug = true; //Should we print the commands we send? Good for debugging
}

/**
   Disable the printing of debug messages
*/
void SFE_ADS122C04::disableDebugging(void)
{
  _printDebug = false; //Turn off extra print statements
}

/**
    Print debug messages using LOG macro.

    \note This print function actually functions as a println
    as LOG includes std::endl at the end of its definition.
*/
void SFE_ADS122C04::debugPrint(char *message)
{
  if (_printDebug == true)
  {
    LOG(message);
  }
}

/**
    Print debug messages using LOG macro.
*/
void SFE_ADS122C04::debugPrintln(char *message)
{
  if (_printDebug == true)
  {
    LOG(message);
  }
}

/**
   Read the temperature from the on-board temperature sensor in centigrade
   (celsius).
   \return The current temperature reading
*/
float SFE_ADS122C04::readPT100Centigrade(void) // Read the temperature in Centigrade
{
  raw_voltage_union raw_v; // union to convert uint32_t to int32_t
  unsigned long start_time = _millis(); // Record the start time so we can timeout
  bool drdy = false; // DRDY (1 == new data is ready)
  float ret_val = 0.0; // Return value
  float RTD, POLY; // Variables needed to convert RTD to Centigrade

  // Start the conversion (assumes we are using single shot mode)
  start();

  // Wait for DRDY to go valid
  while((drdy == false) && (_millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Don't pound the bus too hard
    drdy = checkDataReady();
  }

  // Check if we timed out
  if (drdy == false)
  {
    if (_printDebug == true)
    {
      LOG("readPT100Centigrade: checkDataReady timed out");
    }
    return(ret_val);
  }

  // Read the conversion result
  if(ADS122C04_getConversionData(&raw_v.UINT32) == false)
  {
    if (_printDebug == true)
    {
      LOG("readPT100Centigrade: ADS122C04_getConversionData failed");
    }
    return(ret_val);
  }

  // The raw voltage is in the bottom 24 bits of raw_temp
  // If we just do a <<8 we will multiply the result by 256
  // Instead pad out the MSB with the MS bit of the 24 bits
  // to preserve the two's complement
  if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
  {
    raw_v.UINT32 |= 0xFF000000;
  }

  // raw_v.UINT32 now contains the ADC result, correctly signed
  // Now we need to convert it to temperature using the PT100 resistance,
  // the gain, excitation current and reference resistor value

  // Formulae are taken from:
  // http://www.ti.com/lit/an/sbaa275/sbaa275.pdf
  // https://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf

  // 2^23 is 8388608
  RTD = ((float)raw_v.INT32) / 8388608.0; // Load RTD with the scaled ADC value
  RTD *= PT100_REFERENCE_RESISTOR; // Multiply by the reference resistor
  // Use the correct gain for high and low temperatures
  if ((_wireMode == ADS122C04_4WIRE_HI_TEMP) || (_wireMode == ADS122C04_3WIRE_HI_TEMP) || (_wireMode == ADS122C04_2WIRE_HI_TEMP))
  {
    RTD /= PT100_AMP_GAIN_HI_TEMP; // Divide by the amplifier gain for high temperatures
  }
  else
  {
    RTD /= PT100_AMPLIFIER_GAIN; // Divide by the amplifier gain for low temperatures
  }
  if ((_wireMode == ADS122C04_3WIRE_MODE) || (_wireMode == ADS122C04_3WIRE_HI_TEMP)) // If we are using 3-wire mode
  {
    RTD *= 2.0; // 3-wire mode needs a factor of 2
  }

  // RTD now contains the PT100 resistance in Ohms
  // Now we need to convert this to temperature

  // Calculate the temperature
  ret_val = RTD * -23.10e-9;
  ret_val += 17.5848089e-6;
  ret_val = sqrt(ret_val);
  ret_val -= 3.9083e-3;
  ret_val /= -1.155e-6;

  //  Check if the temperature is positive, return if it is
  if (ret_val >= 0.0)
    return(ret_val);

  // The temperature is negative so we need to use a different formula
  ret_val = -242.02;
  ret_val += 2.2228 * RTD;
  POLY = RTD * RTD; // Load the polynomial with RTD^2
  ret_val += 2.5859e-3 * POLY;
  POLY *= RTD; // Load the polynomial with RTD^3
  ret_val -= 4.8260e-6 * POLY;
  POLY *= RTD; // Load the polynomial with RTD^4
  ret_val -= 2.8183e-8 * POLY;
  POLY *= RTD; // Load the polynomial with RTD^5
  ret_val += 1.5243e-10 * POLY;

  return(ret_val);
}

/**
   Read the temperature in Farenheit.
   \return The current temperature in Farenheit.
*/
float SFE_ADS122C04::readPT100Fahrenheit(void) // Read the temperature in Fahrenheit
{
  return((readPT100Centigrade() * 1.8) + 32.0); // Read Centigrade and convert to Fahrenheit
}

/**
   Read the raw signed 16-bit ADC value as int32_t
   The result needs to be multiplied by VREF / GAIN to convert to Volts
   \param rate The data rate to be used.
   \return The raw voltage (note this will need sign extended).
*/
int32_t SFE_ADS122C04::readRawVoltage(uint8_t rate)
{
  raw_voltage_union raw_v; // union to convert uint32_t to int32_t
  unsigned long start_time = _millis(); // Record the start time so we can timeout
  bool drdy = false; // DRDY (1 == new data is ready)
  uint8_t previousWireMode = _wireMode; // Record the previous wire mode so we can restore it
  uint8_t previousRate = ADS122C04_Reg.reg1.bit.DR; // Record the previous rate so we can restore it

  // Configure the ADS122C04 for raw mode
  // Disable the IDAC, use the internal 2.048V reference and set the gain to 1
  if ((configureADCmode(ADS122C04_RAW_MODE, rate)) == false)
  {
    if (_printDebug == true)
    {
      LOG("readRawVoltage: configureADCmode (1) failed");
    }
    return(0);
  }

  // Start the conversion (assumes we are using single shot mode)
  start();

  // Wait for DRDY to go valid
  while((drdy == false) && (_millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Don't pound the bus too hard
    drdy = checkDataReady();
  }

  // Check if we timed out
  if (drdy == false)
  {
    if (_printDebug == true)
    {
      LOG("readRawVoltage: checkDataReady timed out");
    }
    configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
    return(0);
  }

  // Read the conversion result
  if(ADS122C04_getConversionData(&raw_v.UINT32) == false)
  {
    if (_printDebug == true)
    {
      LOG("readRawVoltage: ADS122C04_getConversionData failed");
    }
    configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
    return(0);
  }

  // Restore the previous wire mode
  if ((configureADCmode(previousWireMode, previousRate)) == false)
  {
  if (_printDebug == true)
    {
      LOG("readRawVoltage: configureADCmode (2) failed");
    }
    return(0);
  }

  // The raw voltage is in the bottom 24 bits of raw_temp
  // If we just do a <<8 we will multiply the result by 256
  // Instead pad out the MSB with the MS bit of the 24 bits
  // to preserve the two's complement
  if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
    raw_v.UINT32 |= 0xFF000000;
  return(raw_v.INT32);
}

/**
   Read the raw signed 16-bit ADC value as uint32_t.
   \warning The ADC data is returned in the least significant 16 bits. The
   caller will need to sign-extend the result.

   \return The latest digital conversion.
*/
uint32_t SFE_ADS122C04::readADC(void)
{
  uint32_t ret_val; // The return value

  // Read the conversion result
  if(ADS122C04_getConversionData(&ret_val) == false)
  {
    if (_printDebug == true)
    {
      LOG("readADC: ADS122C04_getConversionData failed");
    }
    return(0);
  }

  return(ret_val);
}

// Read the internal temperature
/**
   Read the internal temperature
   \param rate The desired data rate.
   \return The internal temperature
*/
float SFE_ADS122C04::readInternalTemperature(uint8_t rate)
{
  internal_temperature_union int_temp; // union to convert uint16_t to int16_t
  uint32_t raw_temp; // The raw temperature from the ADC
  unsigned long start_time = _millis(); // Record the start time so we can timeout
  bool drdy = false; // DRDY (1 == new data is ready)
  float ret_val = 0.0; // The return value
  uint8_t previousWireMode = _wireMode; // Record the previous wire mode so we can restore it
  uint8_t previousRate = ADS122C04_Reg.reg1.bit.DR; // Record the previous rate so we can restore it

  // Enable the internal temperature sensor
  // Reading the ADC value will return the temperature
  if ((configureADCmode(ADS122C04_TEMPERATURE_MODE, rate)) == false)
  {
    if (_printDebug == true)
    {
      LOG("readInternalTemperature: configureADCmode (1) failed");
    }
    return(ret_val);
  }

  // Start the conversion
  start();

  // Wait for DRDY to go valid
  while((drdy == false) && (_millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Don't pound the bus too hard
    drdy = checkDataReady();
  }

  // Check if we timed out
  if (drdy == false)
  {
    if (_printDebug == true)
    {
      LOG("readInternalTemperature: checkDataReady timed out");
    }
    // Attempt to restore the previous wire mode
    configureADCmode(previousWireMode, previousRate);
    return(ret_val);
  }

  // Read the conversion result
  if(ADS122C04_getConversionData(&raw_temp) == false)
  {
    if (_printDebug == true)
    {
      LOG("readInternalTemperature: ADS122C04_getConversionData failed");
    }
    configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
    return(ret_val);
  }

  // Restore the previous wire mode
  if ((configureADCmode(previousWireMode, previousRate)) == false)
  {
  if (_printDebug == true)
    {
      LOG("readInternalTemperature: configureADCmode (2) failed");
    }
    return(ret_val);
  }

  if (_printDebug == true)
  {
    // Output temp as hex to std output
    std::stringstream ss;
    ss <<
      "readInternalTemperature: raw_temp (32-bit) = " << std::hex << raw_temp;
    LOG(ss.str());
  }

  // The temperature is in the top 14 bits of the bottom 24 bits of raw_temp
  int_temp.UINT16 = (uint16_t)(raw_temp >> 10); // Extract the 14-bit value

  // The signed temperature is now in the bottom 14 bits of int_temp.UINT16
  // If we just do a <<2 we will multiply the result by 4
  // Instead we will pad out the two MS bits with the MS bit of the 14 bits
  // to preserve the two's complement
  if ((int_temp.UINT16 & 0x2000) == 0x2000) // Check if the MS bit is 1
  {
    int_temp.UINT16 |= 0xC000; // Value is negative so pad with 1's
  }
  else
  {
    int_temp.UINT16 &= 0x3FFF;  // Value is positive so make sure the two MS bits are 0
  }

  ret_val = ((float)int_temp.INT16) * TEMPERATURE_SENSOR_RESOLUTION; // Convert to float including the 2 bit shift
  return(ret_val);
}


/**
   Configure the input multiplexer
   \param mux_config The desired mux configuration ID, see ADS122C04_ADC_PI.hpp.
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setInputMultiplexer(uint8_t mux_config)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
    return(false);
  ADS122C04_Reg.reg0.bit.MUX = mux_config;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

/**
   Configure the gain
   \param gain_config The desired gain configuration ID, see ADS122C04_ADC_PI.hpp.
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setGain(uint8_t gain_config)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
    return(false);
  ADS122C04_Reg.reg0.bit.GAIN = gain_config;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

/**
   Enable/disable the Programmable Gain Amplifier
   \param enable Enable/disable setting, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::enablePGA(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
    return(false);
  ADS122C04_Reg.reg0.bit.PGA_BYPASS = enable;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

/**
   Set the data rate (sample speed)
   \param rate The desired rate setting, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setDataRate(uint8_t rate)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.DR = rate;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

/**
   Configure the operating mode (normal / turbo)
   \param mode The desired mode setting, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setOperatingMode(uint8_t mode)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.MODE = mode;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

/**
   Configure the conversion mode (single-shot / continuous)
   \param mode The desired conversion mode, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setConversionMode(uint8_t mode)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.CMBIT = mode;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

/**
   Configure the voltage reference
   \param ref Set the voltage reference, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setVoltageReference(uint8_t ref)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.VREF = ref;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

/**
   Enable / disable the internal temperature sensor
   \param enable The temperature sensor configuration value, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::enableInternalTempSensor(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.TS = enable;
  if (_printDebug == true)
  {
    std::stringstream ss;
    ss <<
      "enableInternalTempSensor: ADS122C04_Reg.reg1.bit.TS = " <<
      std::hex << ADS122C04_Reg.reg1.bit.TS;
    LOG(ss.str());
  }
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

/**
   Enable / disable the conversion data counter
   \param enable The data counter configuration, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setDataCounter(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.DCNT = enable;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

/**
   Configure the data integrity check
   \param setting The integrity check configuration, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setDataIntegrityCheck(uint8_t setting)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.CRC = setting;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

/**
   Enable / disable the 10uA burn-out current source
   \param enable The burn-out configuration, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setBurnOutCurrent(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.BCS = enable;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

/**
   Configure the internal programmable current sources
   \param current The internal current configuration, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setIDACcurrent(uint8_t current)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.IDAC = current;
  if (_printDebug == true)
  {
    std::stringstream ss;
    ss <<
      "setIDACcurrent: ADS122C04_Reg.reg2.bit.IDAC = " << std::hex <<
      ADS122C04_Reg.reg2.bit.IDAC;
    LOG(ss.str())
  }
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

/**
   Configure the IDAC1 routing
   \param setting The IDAC1 routing configuration, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setIDAC1mux(uint8_t setting)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) == false)
    return(false);
  ADS122C04_Reg.reg3.bit.I1MUX = setting;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

/**
   Configure the IDAC2 routing
   \param setting The IDAC2 routing configuration, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::setIDAC2mux(uint8_t setting)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) == false)
    return(false);
  ADS122C04_Reg.reg3.bit.I2MUX = setting;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

/**
   Check the DRDY bit (next conversion ready). Data is ready when DRDY is high.
   \return true if DRDY high, false otherwise.
*/
bool SFE_ADS122C04::checkDataReady(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return(ADS122C04_Reg.reg2.bit.DRDY > 0);
}

/**
   Get the input multiplexer configuration, return value can be compared to
   definitions given in ADS122C04_ADC_PI.hpp
   \return The input mux configuration.
*/
uint8_t SFE_ADS122C04::getInputMultiplexer(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
  return(ADS122C04_Reg.reg0.bit.MUX);
}

/**
   Get the gain setting, comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return The gain configuration.
*/
uint8_t SFE_ADS122C04::getGain(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
  return(ADS122C04_Reg.reg0.bit.GAIN);
}

/**
   Get the Programmable Gain Amplifier status, comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return The PGA status configuration.
*/
uint8_t SFE_ADS122C04::getPGAstatus(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
  return(ADS122C04_Reg.reg0.bit.PGA_BYPASS);
}

/**
   Get the data rate (sample speed), comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return The currently configured data rate.
*/
uint8_t SFE_ADS122C04::getDataRate(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return(ADS122C04_Reg.reg1.bit.DR);
}

/**
   Get the operating mode (normal / turbo), comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return The current operating mode
*/
uint8_t SFE_ADS122C04::getOperatingMode(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return(ADS122C04_Reg.reg1.bit.MODE);
}

/**
   Get the conversion mode (single-shot / continuous), comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return The current conversion mode
*/
uint8_t SFE_ADS122C04::getConversionMode(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return(ADS122C04_Reg.reg1.bit.CMBIT);
}

/**
   Get the voltage reference configuration, comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return The voltage reference configuration
*/
uint8_t SFE_ADS122C04::getVoltageReference(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  return(ADS122C04_Reg.reg1.bit.VREF);
}

/**
   Get the internal temperature sensor status, comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return internal temperature sensor status code
*/
uint8_t SFE_ADS122C04::getInternalTempSensorStatus(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
  if (_printDebug == true)
  {

    std::stringstream ss;
    ss <<
      "getInternalTempSensorStatus: ADS122C04_Reg.reg1.bit.TS = " << std::hex <<
      ADS122C04_Reg.reg1.bit.TS;
    LOG(ss.str())
  }
  return(ADS122C04_Reg.reg1.bit.TS);
}

/**
   Get the data counter status code, comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return data counter status
*/
uint8_t SFE_ADS122C04::getDataCounter(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return(ADS122C04_Reg.reg2.bit.DCNT);
}

/**
   Get the data integrity check configuration code, comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return data integrity check configuration
*/
uint8_t SFE_ADS122C04::getDataIntegrityCheck(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return(ADS122C04_Reg.reg2.bit.CRC);
}

/**
   Get the burn-out current status code, comparable to definitions given in ADS122C04_ADC_PI.hpp
   \return the burn-out current status
*/
uint8_t SFE_ADS122C04::getBurnOutCurrent(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return(ADS122C04_Reg.reg2.bit.BCS);
}

/**
   Get the IDAC setting
   \return the current IDAC setting.
*/
uint8_t SFE_ADS122C04::getIDACcurrent(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  if (_printDebug == true)
  {
    std::stringstream ss;
    ss <<
      "getIDACcurrent: ADS122C04_Reg.reg2.bit.IDAC = " << std::hex <<
      ADS122C04_Reg.reg2.bit.IDAC;
    LOG(ss.str());
  }
  return(ADS122C04_Reg.reg2.bit.IDAC);
}


/**
   Get the IDAC1 mux configuration code
   \return IDAC1 mux configuration code
*/
uint8_t SFE_ADS122C04::getIDAC1mux(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
  return(ADS122C04_Reg.reg3.bit.I1MUX);
}

/**
   Get the IDAC2 mux configuration code
   \return IDAC2 mux configuration code
*/
uint8_t SFE_ADS122C04::getIDAC2mux(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
  return(ADS122C04_Reg.reg3.bit.I2MUX);
}

/**
   Update the configuration registers and then initialise the ADC using the 
   supplied parameters.

   \param param Pointer to configuration struct, see ADS122C04_ADC_PI.hpp
   \return true on success, false otherwise
*/
bool SFE_ADS122C04::ADS122C04_init(ADS122C04_initParam *param)
{
  ADS122C04_Reg.reg0.all = 0; // Reset all four register values to the default value of 0x00
  ADS122C04_Reg.reg1.all = 0;
  ADS122C04_Reg.reg2.all = 0;
  ADS122C04_Reg.reg3.all = 0;

  ADS122C04_Reg.reg0.bit.MUX = param->inputMux;
  ADS122C04_Reg.reg0.bit.GAIN = param->gainLevel;
  ADS122C04_Reg.reg0.bit.PGA_BYPASS = param->pgaBypass;

  ADS122C04_Reg.reg1.bit.DR = param->dataRate;
  ADS122C04_Reg.reg1.bit.MODE = param->opMode;
  ADS122C04_Reg.reg1.bit.CMBIT = param->convMode;
  ADS122C04_Reg.reg1.bit.VREF = param->selectVref;
  ADS122C04_Reg.reg1.bit.TS = param->tempSensorEn;

  ADS122C04_Reg.reg2.bit.DCNT = param->dataCounterEn;
  ADS122C04_Reg.reg2.bit.CRC = param->dataCRCen;
  ADS122C04_Reg.reg2.bit.BCS = param->burnOutEn;
  ADS122C04_Reg.reg2.bit.IDAC = param->idacCurrent;

  ADS122C04_Reg.reg3.bit.I1MUX = param->routeIDAC1;
  ADS122C04_Reg.reg3.bit.I2MUX = param->routeIDAC2;

  bool ret_val = true; // Flag to show if the four writeRegs were successful
  // (If any one writeReg returns false, ret_val will be false)
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all);
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all);
  ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all);

  // Read and print the new configuration (if enableDebugging has been called)
  printADS122C04config();

  return(ret_val);
}


/**
   Debug print of the ADS122C04 configuration
*/
void SFE_ADS122C04::printADS122C04config(void)
{
  if (_printDebug == true)
  {
    bool successful = true; // Flag to show if the four readRegs were successful
    // (If any one readReg returns false, success will be false)
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);

    if (successful == false)
    {
      LOG("printADS122C04config: readReg failed");
      return;
    }
    else
    {
      std::stringstream ss;
      ss <<
        "ConfigReg0: MUX=" <<
        ADS122C04_Reg.reg0.bit.MUX <<
        " GAIN=" <<
        ADS122C04_Reg.reg0.bit.GAIN <<
        " PGA_BYPASS=" <<
        ADS122C04_Reg.reg0.bit.PGA_BYPASS;
      LOG(ss.str());
      ss.clear();

      ss <<
        "ConfigReg1: DR=" <<
        ADS122C04_Reg.reg1.bit.DR <<
        " MODE=" <<
        ADS122C04_Reg.reg1.bit.MODE <<
        " CMBIT=" <<
        ADS122C04_Reg.reg1.bit.CMBIT <<
        " VREF="<<
        ADS122C04_Reg.reg1.bit.VREF <<
        " TS=" <<
        ADS122C04_Reg.reg1.bit.TS;
      LOG(ss.str());
      ss.clear();

      ss <<
        "ConfigReg2: DCNT=" <<
        ADS122C04_Reg.reg2.bit.DCNT <<
        " CRC=" <<
        ADS122C04_Reg.reg2.bit.CRC <<
        " BCS=" <<
        ADS122C04_Reg.reg2.bit.BCS <<
        " IDAC=" <<
        ADS122C04_Reg.reg2.bit.IDAC;
      LOG(ss.str());
      ss.clear();

      ss <<
        "ConfigReg3: I1MUX=" <<
        ADS122C04_Reg.reg3.bit.I1MUX <<
        " I2MUX=" <<
        ADS122C04_Reg.reg3.bit.I2MUX;
      LOG(ss.str());
      ss.clear();
    }
  }
}

/**
   Reset the ADC
   \return true on success
*/
bool SFE_ADS122C04::reset(void)
{
  return(ADS122C04_sendCommand(ADS122C04_RESET_CMD));
}

/**
   Start the ADC
   \return true on success
 */
bool SFE_ADS122C04::start(void)
{
  return(ADS122C04_sendCommand(ADS122C04_START_CMD));
}

/**
   Shut down the ADC
   \return true on success
 */
bool SFE_ADS122C04::powerdown(void)
{
  return(ADS122C04_sendCommand(ADS122C04_POWERDOWN_CMD));
}

/**
   Write a value to a specific register on the ADC

   \param reg The target register, see ADS122C04_ADC_PI.hpp.
   \param writeValue The configuration value
   \return true on success
 */
bool SFE_ADS122C04::ADS122C04_writeReg(uint8_t reg, uint8_t writeValue)
{
  uint8_t command = 0;
  command = ADS122C04_WRITE_CMD(reg);
  return(ADS122C04_sendCommandWithValue(command, writeValue));
}

/**
   Read from a specific register.
   \param reg The target register
   \param readValue Pointer into which the value will be written
   \return true on success
*/
bool SFE_ADS122C04::ADS122C04_readReg(uint8_t reg, uint8_t *readValue)
{
  uint8_t command = 0;
  command = ADS122C04_READ_CMD(reg);

  *readValue = i2c_smbus_read_byte_data(_i2c_fd, (uint8_t)command);
  if (*readValue < 0){
    // -ve errno returned if the read failed.
    return false;
  }

  return true;
}

/**
   Send a command to the ADC.
   \param command The desired command, see ADS122C04_ADC_PI.hpp
   \return true on success
*/
bool SFE_ADS122C04::ADS122C04_sendCommand(uint8_t command)
{
  // Return true on success, else false. No way to recover error code.
  return i2c_smbus_write_byte(_i2c_fd, command) == 0;
}

/**
   Send a command with an associated value to the ADC.
   \param command The desired command, see ADS122C04_ADC_PI.hpp
   \param value The associated value.
*/
bool SFE_ADS122C04::ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value)
{
  // Return true on success, else false. No way to recover error code.
  return i2c_smbus_write_byte_data(_i2c_fd, command, value) == 0;
}

/**
   Read the conversion result with count byte. The conversion result is 24-bit
   two's complement (signed) and is returned in the lowest 3 bytes of the
   conversionData parameter. Hence, it will always appear positive and the caller
   is responsible for correctly interpreting this value.

   \warning The working status of this function is unknown for the
   ADS112C04.  If I recall correctly, this will run but given that the
   code pulls three bytes from the I2C read (RXByte), it is likely that
   the returned values are corrupted (attempting to read 24-bits from a
   16-bit value). This function was not used for any practical work.

   \param conversionData Pointer into which the data will be written
   \param count Pointer into which the data count value will be written
   \return true on success
*/
bool SFE_ADS122C04::ADS122C04_getConversionDataWithCount(
                                                         uint32_t *conversionData,
                                                         uint8_t *count
                                                         ){
  uint8_t RXByte_s = 5;
  uint8_t RXByte[RXByte_s] = {0};

  int read = i2c_smbus_read_i2c_block_data(_i2c_fd, ADS122C04_RDATA_CMD, RXByte_s, RXByte);

  if (read < 0){
    LOG("Conversion with count read error. Code: " << read << ".");
    return false;
  }

  // RXByte[0] should be the number of bytes read, which should be 4
  if (RXByte[0] == 3){
    LOG("Only 3 bytes available. Enable DCNT or request"
        " conversion data without count.");
  }

  *count = RXByte[1];
  *conversionData =
    ((uint32_t)RXByte[4]) |
    ((uint32_t)RXByte[3]<<8) |
    ((uint32_t)RXByte[2]<<16);

  return(true);
}

/**
   Read the conversion result. The conversion result is 16-bit
   two's complement (signed) and is returned in the lowest 2 bytes of the
   conversionData parameter. Hence, it will always appear positive and the caller
   is responsible for correctly interpreting this value.

   \note This function is known to work correctly on the ADS112C04.

   \param conversionData Pointer into which the data will be written
   \return true on success
*/
bool SFE_ADS122C04::ADS122C04_getConversionData(uint32_t *conversionData)
{
  uint8_t RXByte_s = 2;
  uint8_t RXByte[RXByte_s] = {0, 0};

  int read = i2c_smbus_read_i2c_block_data(_i2c_fd, ADS122C04_RDATA_CMD, RXByte_s, RXByte);

  if (read < 0){
    LOG("Conversion without count, read error. Code: " << read << ".");
    return false;
  }

  // Otherwise the counter is disabled and the data is in the correct place.
  // Last element should be empty.
  *conversionData =
    ((uint32_t)RXByte[1]) |
    ((uint32_t)RXByte[0]<<8);

  // int secd = (int) *conversionData;//(((int) *conversionData) << 16) >> 16;
  // LOG("Conversion data " << secd);

  return true;
}

/**
   Expose init parameters.
   \return The parameters used on the last initialisation
*/
ADS122C04_initParam* SFE_ADS122C04::getCurrentInitParams(){
  return &lastInitParams;
}

/**
   Update chip configuration. To be called after configuration has been changed.
   \return true on success
*/
bool SFE_ADS122C04::reinitialise(){
  return ADS122C04_init(&lastInitParams);
}
