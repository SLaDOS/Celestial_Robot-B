/**
   \file pol_op_mux.cpp
   \brief ROS node to read from the full eight-unit polarisation sensor.

   This node allows the robot to read from the polarisation array constructed
   for Gkanias et al. (2023). Each polarisation opponent unit is read in sequence
   using the same multiplexer (TCA9548A) as in the wind sensing node
   (i2c_mux.cpp).

   \note Each polarisation opponent unit is uses two I2C analogue to digital
   converters (ADCs). The four photodiodes are split across the two ADCs,
   meaning we need to read from both to read from all four photodiodes.
   Hence, two I2C addresses for one unit.

   \author Robert Mitchell
*/

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h> 
#include <sys/ioctl.h>

#include <fcntl.h> // open() and O_RDWR
#include <unistd.h> // close()
}

#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>

#include "celeste_sensors/ADS122C04_ADC_PI.hpp" // initParams
#include "celeste_sensors/POL_OP.hpp" // POL_OP class wrapper
#include "celeste_sensors/argparse.h" // ArgumentParser

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#define LOG(x) std::cout << x << std::endl

#define MUX_ADDR 0x70 /**< Multiplexer I2C address */
#define A2D_ADDR_1 0x40 /**< 1st ADC address */
#define A2D_ADDR_2 0x41 /**< 2nd ADC address */

argparse::ArgumentParser parser("Parser");

/**
   \brief Initialise the argument parser
   \param parser The parser object.
   \param parser argc Passthrough argc from main.
   \param parser argv Passthrough argv from main.
   \return `true` on successful initialisation, otherwise `false`
*/

bool initParser(std::shared_ptr<rclcpp::Node> node, argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"--gain"})
    .description("Set the PGA gain for the ADCs (3-bit config, see datasheet for"
                 " ADS122C04). Valid values 0-7, gain will be 2^g where g is the"
                 " argument provided here.")
    .required(false);

  parser.add_argument()
    .names({"--rate"})
    .description("Set the conversion rate for the ADCs. Valid values 0-6."
                 " 0 = 20SPS, each increment approximately doubles the"
                 " data rate up to 6 = 1000SPS.")
    .required(false);

  parser.add_argument()
    .names({"--bus"})
    .description("Set the i2c bus number(0-1).")
    .required(false);

  parser.add_argument()
    .names({"-n","--number_of_units"})
    .description("The number of POL-OP units connected to the mux (1 - 8).")
    .required(false);

  parser.enable_help();

  auto err = parser.parse(argc, const_cast<const char**>(argv));
  if (err) {
    std::stringstream ss;
    ss<<err;
    RCLCPP_ERROR(node->get_logger(), "argparse error: %s", ss.str().c_str());
    return false;
  }

  return true;
}

/**
   Initialises the multiplexer on the I2C bus. If the initialisation fails then 
   the node will shut down.

   \param bus The string identifier for the I2C bus, see Linux kernel documentation.
   \return A file descriptor which can be used to access the multiplexer.
*/
int init_mux(std::string bus){
  int fd;

  if ((fd = open(bus.c_str(), O_RDWR)) < 0) std::exit(1);

  if (ioctl(fd, I2C_SLAVE, MUX_ADDR) < 0){
    close(fd);
    std::exit(1);
  }

  return fd;
}

/**
   Selects a given channel on the multiplexer and delays to allow time for
   channel selection to take place.

   Will print a message to stdout if the channel selection fails.

   \param channel The channel (0 - 7) to read from.
   \param fd The file descriptor used to communicate with the mux
             on the I2C bus.
   \note This function does not return any value despite its return type.
 */
bool select_channel(const uint8_t channel, const int fd){
  int ret = i2c_smbus_write_byte_data(fd, 0x00, (1 << channel));
  if (ret == -1) {LOG("Failed to select channel");}
  // Allow time for channel switch
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

/**
    Main loop.
*/
int main(int argc, char **argv){
  /* Initialise the ROS node */
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("i2c_mux_pol_op");

  /* Initialise argparser and set specified parameters. */
  if(!initParser(node, parser, argc, argv)){
    rclcpp::shutdown();
    return -1;
  }
  if(parser.exists("help")){
    parser.print_help();
    rclcpp::shutdown();
    return 0;
  }

  int pga_gain =
    parser.exists("gain") ?
    parser.get<int>("gain") :
    0;

  int data_rate =
    parser.exists("rate") ?
    parser.get<int>("rate") :
    0;

  int n_pol_ops =
    parser.exists("number_of_units") ?
    parser.get<int>("number_of_units") :
    1;

  int i2c_bus = 
    parser.exists("bus") ?
    parser.get<int>("bus") :
    1;
    
  RCLCPP_INFO(node->get_logger(),"Gain setting: %d", pga_gain);
  RCLCPP_INFO(node->get_logger(),"Number of POL-OPs: %d", n_pol_ops);

  /* Initialise the mux */
  std::string bus = "/dev/i2c-";
  bus += std::to_string(i2c_bus);
  int fd = init_mux(bus);

  /*
     Because the number of unit is configurable, we generate two parallel
     vectors of Publishers and POL_OP objects (i.e. pubs[i] is the Publisher
     for data read from pol_ops[i]). These vectors are grown to the correct
     size.
  */
  std::vector<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr> pubs;
  std::vector<POL_OP> pol_ops;

  /* Initialise each unit and its Publisher */
  for (int i = 0; i < n_pol_ops; i++){
    select_channel(i, fd);
    POL_OP poi(bus, A2D_ADDR_1, A2D_ADDR_2, false, i);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::stringstream name;
    
    if (i2c_bus == 0) {
      name << "pol_op_" << i+4;
    }
    else{
      name << "pol_op_" << i;
    };

    auto poi_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>(name.str().c_str(), rclcpp::SensorDataQoS());
    pubs.push_back(poi_pub);
    pol_ops.push_back(poi);
  }


  /* Set custom options for each pol_op unit */
  for (int i = 0; i < n_pol_ops; i++){
    pol_ops[i].set_gain(pga_gain);
    pol_ops[i].set_data_rate(data_rate);
  }


  /*
     Read from each unit in sequence.
     Note that the timing code and related output was used to characterise
     sensor latency and does not have any function.
  */
  rclcpp::Rate loop_rate(10);
  auto start = std::chrono::system_clock::now();
  while(rclcpp::ok()) {
    int s_readings = 4;
    int readings[s_readings];
    auto full_read_start = std::chrono::system_clock::now();
    for (int i = 0; i < n_pol_ops; i++){

      for (int j = 0; j < s_readings; j++) readings[j] = 0; // Clear readings
      // Read from pol_op
      select_channel(i, fd); //This will sleep for 10 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      // Min working delay = 25; 11ms was used for pol experiments
      // before this was known.
      int delay;
      if (i == n_pol_ops-1){
        // The last sensor needs more delay, otherwise photodiodes' values will be the same 
        delay = 25;
        std::cout << "more time"<< std::endl;
      }else{
        delay = 11;
      }
      bool success = pol_ops[i].read_sensor_interleaved(readings, delay);

      std::vector<int> res(readings, readings+s_readings);

      // Publish photodiode data on the correct topic.
      std_msgs::msg::Int32MultiArray msg;
      msg.data = res;
      auto time = std::chrono::system_clock::now() - start;
      auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time);
      std::cout << "Time since last publication: " << millis.count() << "ms" << std::endl;
      pubs[i]->publish(msg);
      start = std::chrono::system_clock::now();
    }


    auto full_read_time = std::chrono::system_clock::now() - full_read_start;
    auto full_read_millis = std::chrono::duration_cast<std::chrono::milliseconds>(full_read_time);
    std::cout << "Full read time: " << full_read_millis.count() << "ms" << std::endl;

    rclcpp::spin_some(node)	;
    loop_rate.sleep();
  }
  LOG("Exiting.");
  return 0;
}
