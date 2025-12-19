#ifndef MECANUMDRIVE_ESP_CONFIG_H
#define MECANUMDRIVE_ESP_CONFIG_H

#include <string>

struct Config
{
  // Wheel Names (ROS2 Parameters)
  std::string left_front_wheel_name = "front_left_wheel_joint";
  std::string right_front_wheel_name = "front_right_wheel_joint";
  std::string left_rear_wheel_name = "rear_left_wheel_joint";
  std::string right_rear_wheel_name = "rear_right_wheel_joint";
  
  // Encoder Calibration (individual counts per revolution for each wheel)
  int left_front_enc_counts_per_rev = 5037;
  int right_front_enc_counts_per_rev = 2670;
  int left_rear_enc_counts_per_rev = 2689;
  int right_rear_enc_counts_per_rev = 1982;
  
  // Serial Port Config
  float loop_rate = 30.0;
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 1000;
};

#endif // MECANUMDRIVE_ESP_CONFIG_H