/**
   \file: AMS_5600_PI.cpp
   \brief Implementation of AMS_5600 class.

   File provides the implementation of the AMS_5600 class for reading
   from the AS5600 magnetic encoder on the Raspberry Pi.

   The original version (written for Arduino) can be
   found at:
   https://github.com/Seeed-Studio/Seeed_Arduino_AS5600

   Requires i2c-tools, libi2c-dev.
*/

// I2C includes
extern "C" {
// #include <linux/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
#include <fcntl.h> // open() and O_RDWR
#include <unistd.h> // close()
}

#include <string> // String utilities
#include <chrono> // Time durations
#include <thread> // Sleep
#include <cmath> // std::abs


#include "celeste_sensors/AMS_5600_PI.hpp"

/**
   Default constructor
*/
AMS_5600::AMS_5600()
{
  /* set i2c address */
  _ams5600_Address = 0x36;

  /* load register values*/
  /* c++ class forbids pre loading of variables */
  _zmco = 0x00;
  _zpos_hi = 0x01;
  _zpos_lo = 0x02;
  _mpos_hi = 0x03;
  _mpos_lo = 0x04;
  _mang_hi = 0x05;
  _mang_lo = 0x06;
  _conf_hi = 0x07;
  _conf_lo = 0x08;
  _raw_ang_hi = 0x0c;
  _raw_ang_lo = 0x0d;
  _ang_hi = 0x0e;
  _ang_lo = 0x0f;
  _stat = 0x0b;
  _agc = 0x1a;
  _mag_hi = 0x1b;
  _mag_lo = 0x1c;
  _burn = 0xff;
}

/**
   Custom constructor which also allows the multiplexer channel
   to be stored alongside the rest of the object properties.

   \param channel The mux channel to which this device is connected.
 */
AMS_5600::AMS_5600(uint8_t channel){
  _channel = channel; // Only additional step, useful for the mux

    /* set i2c address */
  _ams5600_Address = 0x36;

  /* load register values*/
  /* c++ class forbids pre loading of variables */
  _zmco = 0x00;
  _zpos_hi = 0x01;
  _zpos_lo = 0x02;
  _mpos_hi = 0x03;
  _mpos_lo = 0x04;
  _mang_hi = 0x05;
  _mang_lo = 0x06;
  _conf_hi = 0x07;
  _conf_lo = 0x08;
  _raw_ang_hi = 0x0c;
  _raw_ang_lo = 0x0d;
  _ang_hi = 0x0e;
  _ang_lo = 0x0f;
  _stat = 0x0b;
  _agc = 0x1a;
  _mag_hi = 0x1b;
  _mag_lo = 0x1c;
  _burn = 0xff;
}


/**
   Get the mux channel associated with this device.
   \return The channel to which the device is connected.
*/
const uint8_t AMS_5600::getChannel(){ return _channel; }

/**
   Initialise the I2C interface. See https://www.kernel.org/doc/Documentation/i2c/dev-interface

   \param bus The string identifier of the desired bus.
*/
void AMS_5600::init(std::string bus){
  _i2c_fd = 0;

  if ((_i2c_fd = open(bus.c_str(), O_RDWR)) < 0) std::exit(1);

  if (ioctl(_i2c_fd, I2C_SLAVE, _ams5600_Address) < 0){
      close(_i2c_fd);
      std::exit(1);
  }
}


/**
   Set output mode. 0=PWM, 1=analog (full range from 0% to 100% between GND and VDD).

   \param mode The mode setting, 0 for PWM, 1 for analog.
*/
void AMS_5600::setOutPut(uint8_t mode){
    uint8_t config_status;

    config_status = readOneByte(_conf_lo);

    if (mode == 1) {
        config_status = config_status & 0xcf;
    } else {
        config_status = config_status & 0xef;
    }

    writeOneByte(_conf_lo, lowByte(config_status));
}

/**
   Get the I2C address of the device.
   \return The I2C address.
*/
int AMS_5600::getAddress(){
  return _ams5600_Address;
}

/**
   Set the value stored in the maximum angle register.
   \param newMaxAngle the desired maximum angle
   \return the value in the maximum angle register
   \note If -1 is passed as newMaxAngle, then this method
   will return the raw angle of the magnet.
*/
word AMS_5600::setMaxAngle(word newMaxAngle){
  word retVal;
  if(newMaxAngle == -1)
  {
    _maxAngle = getRawAngle();
  }
  else
    _maxAngle = newMaxAngle;

  writeOneByte(_mang_hi, highByte(_maxAngle));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  writeOneByte(_mang_lo, lowByte(_maxAngle));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  retVal = readTwoBytes(_mang_hi, _mang_lo);
  return retVal;
}


/**
   Get the value in the max angle register.
   \return The current max angle.
 */
word AMS_5600::getMaxAngle()
{
  return readTwoBytes(_mang_hi, _mang_lo);
}

/**
   Set the encoder start position.
   \param startAngle The new start position.
   \return The value of the start position register.
   \note If -1 is passed as startAngle, then this method
   will return the raw angle of the magnet.
*/
word AMS_5600::setStartPosition(word startAngle)
{
  if(startAngle == -1)
  {
    _rawStartAngle = getRawAngle();
  }
  else
    _rawStartAngle = startAngle;

  writeOneByte(_zpos_hi, highByte(_rawStartAngle));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  writeOneByte(_zpos_lo, lowByte(_rawStartAngle));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  _zPosition = readTwoBytes(_zpos_hi, _zpos_lo);

  return(_zPosition);
}

/**
   Get the current encoder start position.
   \return The current start position.
*/
word AMS_5600::getStartPosition()
{
  return readTwoBytes(_zpos_hi, _zpos_lo);
}

/**
   Set the encoder end position.
   \param endAngle The new end position.
   \return The value of the end position register.
   \note If -1 is passed as endAngle, then this method
   will return the raw angle of the magnet.
*/
word AMS_5600::setEndPosition(word endAngle)
{
  if(endAngle == -1)
    _rawEndAngle = getRawAngle();
  else
    _rawEndAngle = endAngle;

  writeOneByte(_mpos_hi, highByte(_rawEndAngle));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  writeOneByte(_mpos_lo, lowByte(_rawEndAngle));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  _mPosition = readTwoBytes(_mpos_hi, _mpos_lo);

  return(_mPosition);
}

/**
   Get the current encoder end position.
   \return The current end position.
*/
word AMS_5600::getEndPosition()
{
  word retVal = readTwoBytes(_mpos_hi, _mpos_lo);
  return retVal;
}

/**
   Get the raw angle of the magnet.
   \return The angle of the magnet w.r.t. the sensor.
   \note This method ignores start, end, and max angle
   settings.
*/
word AMS_5600::getRawAngle()
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);
}

/**
   Measure the rotation speed. Takes the absolute angular 
   difference between two measurements taken 10ms apart.
   \return The rotation speed of the magnet.
   \warning This method is meant to return speed in degrees
   per second but this has not been checked or calibrated. 
   You \b must check the speed measurement if comparison to other
   devices/data is required.
 */
double AMS_5600::getSpeed()
{
  double first_measurement, second_measurement;
  first_measurement = this->getRawAngle()*0.087;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  second_measurement = this->getRawAngle()*0.087;

  return std::fabs(second_measurement - first_measurement) * 10;
}


/**
   Gets the scaled angle (angle w.r.t. start, end, and max value settings).
   \return The scaled angle.
*/
word AMS_5600::getScaledAngle()
{
  return readTwoBytes(_ang_hi, _ang_lo);
}

/**
   Check for a magnet in range of the encoder.
   \return 1 if magnet is present, 0 if not.
*/
int AMS_5600::detectMagnet()
{
  int magStatus;
  int retVal = 0;
  /*0 0 MD ML MH 0 0 0*/
  /* MD high = AGC minimum overflow, Magnet to strong */
  /* ML high = AGC Maximum overflow, magnet to weak*/
  /* MH high = magnet detected*/
  magStatus = readOneByte(_stat);

  if(magStatus & 0x20)
    retVal = 1;

  return retVal;
}

/**
   Get the magnet strength. 
   \verbatim
   0 == no magnet
   1 == too weak
   2 == just right
   3 == too strong
   \endverbatim
   \return The strength code (0, 1, 2, or 3).
*/
int AMS_5600::getMagnetStrength()
{
  int magStatus;
  int retVal = 0;
  /*0 0 MD ML MH 0 0 0*/
  /* MD high = AGC minimum overflow, Magnet too strong */
  /* ML high = AGC Maximum overflow, magnet too weak*/
  /* MH high = magnet detected*/
  magStatus = readOneByte(_stat);
  if(detectMagnet() ==1)
  {
      retVal = 2; /*just right */
      if(magStatus & 0x10)
        retVal = 1; /*too weak */
      else if(magStatus & 0x08)
        retVal = 3; /*too strong */
  }

  return retVal;
}

/**
   Get the value in the AGC register.
   \return The value in the AGC register.
*/
int AMS_5600::getAgc()
{
  return readOneByte(_agc);
}

/**
   Get the value in the magnitude register.
   \return The value in the magnitude register
n*/
word AMS_5600::getMagnitude()
{
  return readTwoBytes(_mag_hi, _mag_lo);
}

/**
   Determine how many times the chip has been permenantly written
   to (the burn count).
   \return The burn count.
*/
int AMS_5600::getBurnCount()
{
  return readOneByte(_zmco);
}

/**
   Permenantly burn the start and end positions onto the chip.
   The return codes are:
   \verbatim
    1 == success
   -1 == no magnet
   -2 == burn limit exceeded
   -3 == start and end positions not set (useless burn)
   \endverbatim
   \return Success code

   \note If the return code is negative, then no burn will be attempted
   (see source code).

   \warning This method will \b permenantly burn the scaling settings onto
   the chip. This can be done a maximum of three times. Make sure you have
   a good reason before altering the chip.
*/
int AMS_5600::burnAngle()
{
  int retVal = 1;
  _zPosition = getStartPosition();
  _mPosition = getEndPosition();
  _maxAngle  = getMaxAngle();

  if(detectMagnet() == 1)
  {
    if(getBurnCount() < 3)
    {
      if((_zPosition == 0)&&(_mPosition ==0))
        retVal = -3;
      else
        writeOneByte(_burn, 0x80);
    }
    else
      retVal = -2;
  }
  else
    retVal = -1;

  return retVal;
}

/*
  Burn the max angle and configuration data to the chip.

  Return codes:
  \verbatim
    1 == success
   -1 == burn limit exceeded
   -2 == max angle is too small must be at least 18 degrees.
   \endverbatim
  \return Success code.
  \warning This can be done \b once. Make sure you have a good
  reason for permenantly altering the chip.

  \note Both this and burnAngle use the same burn counter but
  can be set a different number of times. If you must burn something
  to the chip then do the max angle and config first.
*/
int AMS_5600::burnMaxAngleAndConfig()
{
  int retVal = 1;
  _maxAngle  = getMaxAngle();

  if(getBurnCount() ==0)
  {
    if(_maxAngle*0.087 < 18) {
      retVal = -2;
    } else {
      writeOneByte(_burn, 0x40);
    }
  } else {
    retVal = -1;
  }

  return retVal;
}


/**
   Read one byte from a specified register.
   \param in_adr The register address from which to read
   \return The data read from the I2C bus.
   \note You \b must call init before attempting to interact with the I2C
   bus (read or write).
*/
int AMS_5600::readOneByte(int in_adr){
  int retVal = -1;

  /* Arduino - Manual I2C read procedure:
     Wire.beginTransmission(_ams5600_Address); // i2c_start with device address
     Wire.write(in_adr); // write register address that we want to read from
     Wire.endTransmission(); // i2c_stop
     Wire.requestFrom(_ams5600_Address, 1); // Read a byte from the address
  */

  // Relies on in_adr being a sensible value (i.e. uint8_t).
  retVal = i2c_smbus_read_byte_data(_i2c_fd, in_adr);

  return retVal;
}

/**
   Read two bytes from the given register range.
   \param in_adr_hi The high end of the address range.
   \param in_adr_lo The low end of the address range.
   \return The data read from the I2C bus.
   \note You \b must call init before attempting to interact with the I2C
   bus (read or write).
*/
word AMS_5600::readTwoBytes(int in_adr_hi, int in_adr_lo)
{
  word retVal = -1;

  /* Read Low Byte */
  int low = i2c_smbus_read_byte_data(_i2c_fd, in_adr_lo);

  /* Read High Byte */
  int high = i2c_smbus_read_byte_data(_i2c_fd, in_adr_hi);

  // Combine high and low into single 2-byte value
  high = high << 8;
  retVal = high | low;

  return retVal;
}

/**
   Write one byte to a specified register.
   \param adr_in The target register.
   \param dat_in The data to write.
   \note You \b must call init before attempting to interact with the I2C
   bus (read or write).
   \warning The caller must ensure that dat_in is no more than one byte in 
   size. If the data is larger than one byte then the behaviour is not known. 
*/
void AMS_5600::writeOneByte(int adr_in, int dat_in)
{
  i2c_smbus_write_byte_data(_i2c_fd, adr_in, dat_in);
}

/**
   Extract the low byte of a standard int.
   \param w The full integer.
   \return bits 0-7
*/
inline uint8_t AMS_5600::lowByte(int w) {
  return ((uint8_t) w & 0xff);
}

/**
   Extract the 'high' byte of a standard int.
   \note This method returns bits 8-15 of a standard four byte integer.
   It does \b not return the eight most significant bits.
   \param w The full integer.
   \return bits 8-15
*/
inline uint8_t AMS_5600::highByte(int w) {
  return ((uint8_t) w >> 8);
}

