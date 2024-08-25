/**
   \file POL_OP.cpp
   \brief Implementation of the POL_OP wrapper class for reading the polarisation 
          opponent PCBs. Modified to only read the last two photodiodes.

   The polarisation opponent PCBs contain two analogue-to-digital converters
   which can be read directly, however, this class was constructed to abstract
   this process to a 'polarisation opponent unit' or POL_OP.

   See POL_OP class documentation.

   \note The ADC we use is the ADS112C04 by Texas Instruments. The
   library for the ADC was adapted from a previous Arduino library for
   the ADS122C04 (112 vs. 122). As a result, the device is erroneously
   called the ADS122C04 throughout the code. I was not able to fix
   this without introducing major changes (which invariably introduce
   major bugs); I was worried about breaking something which I then
   wouldn't have time to test. Please be aware of this difference
   if you plan to modify any of the I2C libraries for this device!

   Original written by: Robert Mitchell (refmitchell)
   Modified by: Nash Xu (SLaDOS)
*/

#include "celeste_sensors/POL_OP.hpp"

#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <bitset>

#define LOG(x) std::cout << x << std::endl;

/**
   Constructor. Initialises both ADCs on the board.

   \param i2c_bus The string ID of the I2C bus.
   \param A2D1_addr The I2C address of the first ADC.
   \param A2D2_addr The I2C address of the second ADC.
   \param debug Debug flag, passed to the ADC library.
   \param channel The I2C multiplexer which this unit is connected to.
*/
POL_OP::POL_OP(std::string i2c_bus,
               uint8_t A2D1_addr,
               uint8_t A2D2_addr,
               bool debug,
               uint8_t channel){
  uint8_t A2D_addrs[2] = {A2D1_addr, A2D2_addr};

  for (int i = 0; i < 2; i++){
    // Debugging output disabled by default
    if (debug)
      A2Ds[i].enableDebugging();

    // Initialise the A2Ds
    if(!(A2Ds[i].begin(A2D_addrs[i], i2c_bus))){
      LOG("A2D" << i+1 << " initialisation error. Exiting.");
      exit(1);
    }

    uint8_t rate = 1; //Default 0; 3 bit code is used to select the datarate, see datasheet
    if(!(A2Ds[i].configureADCmode(ADS122C04_POL_OP_MODE, rate))){
      LOG("A2D" << i+1 << " mode configuration error. Exiting.");
      exit(1);
    }

    // Start the ADC. Needed for continuous mode, redundant for single-shot
    A2Ds[i].start();
    A2Ds[i].setInputMultiplexer(ADS122C04_MUX_AIN1_AIN0); //Initialise mux channel
  }

  LOG("POL_OP: Init success");

  // Set mux channel
  this->channel = channel;
}

/**
   Read raw data from a single ADC channel.

   \note In addition to the Adafruit I2C mux, each ADC also has its own
   multiplexer, on which two channels are in use (one for each
   photodiode). This function assumes the caller has already
   configured the ADC to read from the desired channel.

   \warning This function was used to read from the polarisation opponent
   units during early prototyping and its current working status is unknown.

   \warning The ADC readout uses a 16-bit encoding which requires sign
   extension which is not provided by this function!

   \param A2D The ADC object we want to read from.
   \param delay A time delay (in ms) used to ensure data integrity.
   \return The raw 16 bit ADC data.
*/
uint32_t POL_OP::read_raw_data(SFE_ADS122C04 A2D, uint8_t delay){
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  A2D.start(); // Start conversion

  // Record the start time so we can timeout
  // _millis() provided by A2D header, should fix this (bb_util)
  unsigned long start_time = _millis();
  bool drdy = false;

  while ((drdy == false) &&
         (_millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT))
         ){

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    drdy = A2D.checkDataReady();
  }

  if (drdy == false) {
    LOG("DRDY timed out.");
    return 0; // Error codes?
  }

  // Read raw data
  int32_t raw_ADC_data = A2D.readADC();
  return raw_ADC_data;
}

/**
   Read from the ADC if it is configured to continuous mode.

   \note You must make sure the ADC is in continuous mode before
   assuming this code will work. If initialised in POL_OP_MODE, this
   will be the case. Similarly, you must make sure the ADC channel is
   set prior to attempting a read to ensure sensible data.

   \param A2D The ADC object from which we wish to read.
   \param delay A time delay (in ms) to ensure data integrity.
   \return The raw ADC reading.
*/
uint32_t POL_OP::read_continuous(SFE_ADS122C04 A2D, uint8_t delay){
  // The data counter is not in use. This function assumes that the
  // reading will be 'new'.
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  int32_t raw_ADC_data = A2D.readADC();
  return raw_ADC_data;
}

/**
   This function performs one complete read cycle of a specific ADC
   on the PCB.

   \param idx The integer index of the ADC to be read (0 or 1).
   \param readings A pointer to an array large enough to store the output
                   (of size > 2). The readings will be placed in this array.
   \param delay A time delay (in ms) used to ensure data integrity.

   \return True on success, false otherwise.

   \warning The current working status of this function is unknown.
*/
bool POL_OP::read_A2D(uint8_t idx, uint32_t* readings, uint8_t delay){
  /*
    Performs one complete read cycle of a specific A2D.
    Results are stored in readings, which should be
    declared by the caller.

    Output array should be large enough for both results.
   */
  if (idx > N_A2Ds - 1) {
    LOG("A2D read err, A2D idx out of range.");
    return false;
  }

  A2Ds[idx].setInputMultiplexer(ADS122C04_MUX_AIN0_AIN1);
  readings[0] = read_continuous(A2Ds[idx], delay);
  A2Ds[idx].setInputMultiplexer(ADS122C04_MUX_AIN2_AIN3);
  readings[1] = read_continuous(A2Ds[idx], delay);

  return true;
}

/**
   Perform a full interleaved read from the PCB.

   This PCB read function takes advantage of the delays incurred in
   channel switching on the ADCs by performing one read, changing channel,
   then reading from the other ADC while we wait for the first one to
   finish changing channel.

   \parblock
   \note In the source there is a lot of code dedicated to time tracking.
   This is functionally irrelevant and was only used to characterise the
   sensor read latency. All print-outs should currently be disabled and
   can be re-enabled by uncommenting the print stream below.
   \endparblock

   \parblock
   \note The default delay is 100ms but this can be set as low as 25ms.
         Any delay less than 25ms will start to cause read errors for the
         last polarisation opponent unit in the chain.
   \endparblock

   \param readings Pointer to an array large enough to house all four
                   photodiode readings as integers. The photodiode readings
                   will be stored here.
   \param delay Time delay (in ms) used to ensure data integrity.
   \return true on success, false otherwise
*/
bool POL_OP::read_sensor_interleaved(int *readings, uint8_t delay){
  uint32_t out_2[2] = {0,0};

  int read_times[4] = {0,0,0,0};

  auto start = std::chrono::system_clock::now();
  out_2[0] = read_continuous(A2Ds[1], delay*0);
  auto time = std::chrono::system_clock::now() - start;
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time);

  A2Ds[1].setInputMultiplexer(ADS122C04_MUX_AIN2_AIN3);
  
  start = std::chrono::system_clock::now();
  out_2[1] = read_continuous(A2Ds[1], delay*2);
  time = std::chrono::system_clock::now() - start;
  millis = std::chrono::duration_cast<std::chrono::milliseconds>(time);
  read_times[3] = (int) millis.count();

  // Reset multiplexers to starting position
  A2Ds[1].setInputMultiplexer(ADS122C04_MUX_AIN1_AIN0);

  std::stringstream ss;
  ss << "PD read times: [ ";
  for (int i = 0; i < 4; i++){
    if (i < 3)
      ss << read_times[i] << "ms, ";
    else
      ss << read_times[i] << "ms ] " << std::endl;
  }

  for (int i = 0; i < N_A2Ds; i++){
    // Sign-extended readings
    readings[i+2] =(((int) out_2[i]) << 16) >> 16;
  }
  std::bitset<32> c_read = readings[0];
  std::bitset<32> c_read_1 = readings[1];

  return true;
}

/**
   Get the I2C mux channel information for this polarisation opponent unit.

   \note This is referring to the sensor-level mux (i.e. the Adafruit one),
   \b not the ADC muxes.p

   \return The I2C mux channel to which this pol op unit is connected.
*/
uint8_t POL_OP::get_channel(){ return this->channel; }

/**
  Set the PGA gain for both ADCs on the PCB.

  \note The gain must be a valid 3-bit gain setting for the
  ADS122C04. Use relevant definitions in ADS122C04_ADC_PI.hpp
  (ADS122C04_GAIN_X) to ensure valid settings are used. No error
  checking is included!

  \param The desired 3-bit gain setting.

  \return true on success, false on failure
 */
bool POL_OP::set_gain(uint8_t gain){
  bool ret = true;
  for (int i = 0; i < N_A2Ds; i++) {
    // For each A2D get the most recent initParams, update the
    // gain and then reinitialise the chip.
    ADS122C04_initParam* initParams = A2Ds[i].getCurrentInitParams();
    initParams->gainLevel = gain;
    ret = ret && A2Ds[i].reinitialise();
    if (!ret) LOG("Reinitialisation error with ADC " << i+1);
  }

  if (ret) LOG("POL_OP: Gain set to " << (int) gain << ".");
  return ret;
}


/**
   Set data rate using presets defined in ADS122_ADC_PI.hpp;
   definitions included below for convenience:

   \verbatim
   #define ADS122C04_DATA_RATE_20SPS   0x0
   #define ADS122C04_DATA_RATE_45SPS   0x1
   #define ADS122C04_DATA_RATE_90SPS   0x2
   #define ADS122C04_DATA_RATE_175SPS  0x3
   #define ADS122C04_DATA_RATE_330SPS  0x4
   #define ADS122C04_DATA_RATE_600SPS  0x5
   #define ADS122C04_DATA_RATE_1000SPS 0x6
   \endverbatim

   See the datasheet for the \b ADS112C04 for more information.

   \param The desired data rate.
   \return true on success, false on failure.
*/
bool POL_OP::set_data_rate(uint8_t rate){
  bool ret = true;
  for (int i = 0; i < N_A2Ds; i++) {
    // For each A2D get the most recent initParams, update the
    // gain and then reinitialise the chip.
    ADS122C04_initParam* initParams = A2Ds[i].getCurrentInitParams();
    initParams->dataRate = rate;
    ret = ret && A2Ds[i].reinitialise();
    if (!ret) LOG("Reinitialisation error with ADC " << i+1);
  }

  if (ret) LOG("POL_OP: Data rate set to " << (int) rate << ".");
  return ret;
}
