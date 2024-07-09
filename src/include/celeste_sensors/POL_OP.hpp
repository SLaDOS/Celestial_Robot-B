/**
   \file POL_OP.hpp
   \brief Declaration of the POL_OP wrapper class for reading the
          polarisation oppponent PCBs.

   See POL_OP class documentation.
*/

#ifndef POL_OP_H
#define POL_OP_H

#include "celeste_sensors/ADS122C04_ADC_PI.hpp"

#include <cstdint>
#include <string>

#define N_A2Ds 2 /**< Number of ADCs on the polarisation opponent PCVB */

class POL_OP
{
private:
  uint8_t channel = 0;
  SFE_ADS122C04 A2Ds[2];

  uint32_t read_raw_data(SFE_ADS122C04 A2D, uint8_t delay=100);
  uint32_t read_continuous(SFE_ADS122C04 A2D, uint8_t delay=100);

public:
  POL_OP(std::string i2c_bus = "/dev/i2c-1",
         uint8_t A2D1_addr = 0x40,
         uint8_t A2D2_addr = 0x41,
         bool debug = false,
         uint8_t channel = 0);

  bool read_sensor(int *readings, uint8_t delay=100);
  bool read_sensor_interleaved(int *readings, uint8_t delay=100);
  bool read_A2D(uint8_t idx, uint32_t* readings, uint8_t delay=100);
  uint8_t get_channel();
  bool set_gain(uint8_t gain);
  bool set_data_rate(uint8_t rate);
};

#endif
