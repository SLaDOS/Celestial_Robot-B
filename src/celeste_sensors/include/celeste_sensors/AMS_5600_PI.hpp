/**
   \file: AMS_5600_PI.hpp
   \brief Declaration of AMS_5600 class.

   File provides the declaration of the AMS_5600 class for reading
   from the AS5600 magnetic encoder on the Raspberry Pi.

   The original version (written for Arduino) can be
   found at:
   https://github.com/Seeed-Studio/Seeed_Arduino_AS5600

   Requires i2c-tools, libi2c-dev.
*/

#ifndef AMS_5600_PI_h
#define AMS_5600_PI_h

typedef uint16_t word;

class AMS_5600
{
public:
  AMS_5600(void);
  AMS_5600(uint8_t channel);
  int getAddress();

  word setMaxAngle(word newMaxAngle = -1);
  word getMaxAngle();

  word setStartPosition(word startAngle = -1);
  word getStartPosition();

  word setEndPosition(word endAngle = -1);
  word getEndPosition();

  word getRawAngle();
  word getScaledAngle();

  double getSpeed();

  int  detectMagnet();
  int  getMagnetStrength();
  int  getAgc();
  word getMagnitude();
  int  getBurnCount();
  int  burnAngle();
  int  burnMaxAngleAndConfig();
  void setOutPut(uint8_t mode);

  std::string testInteraction();
  const uint8_t getChannel();

  // Initialisation stage required for general linux i2c
  void init(std::string);

private:
  std::string _i2c_bus_filename;
  int _i2c_fd = 0;
  uint8_t _channel = 0;

  int _ams5600_Address;
  word _rawStartAngle;
  word _zPosition;
  word _rawEndAngle;
  word _mPosition;
  word _maxAngle;

  /* Registers */
  int _zmco;
  int _zpos_hi;    /*zpos[11:8] high nibble  START POSITION */
  int _zpos_lo;    /*zpos[7:0] */
  int _mpos_hi;    /*mpos[11:8] high nibble  STOP POSITION */
  int _mpos_lo;    /*mpos[7:0] */
  int _mang_hi;    /*mang[11:8] high nibble  MAXIMUM ANGLE */
  int _mang_lo;    /*mang[7:0] */
  int _conf_hi;
  int _conf_lo;
  int _raw_ang_hi;
  int _raw_ang_lo;
  int _ang_hi;
  int _ang_lo;
  int _stat;
  int _agc;
  int _mag_hi;
  int _mag_lo;
  int _burn;

  int readOneByte(int in_adr);
  word readTwoBytes(int in_adr_hi, int in_adr_lo);
  void writeOneByte(int adr_in, int dat_in);


  // Replicate highByte/lowByte functions in Arduino.h
  inline uint8_t lowByte(int);
  inline uint8_t highByte(int);


};
#endif
