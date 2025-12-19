#ifndef MECANUMDRIVE_ESP_REAL_ROBOT_H
#define MECANUMDRIVE_ESP_REAL_ROBOT_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "hardware/config.h"
#include "hardware/wheel.h"
#include "hardware/esp_comms.h"

using hardware_interface::return_type;

class MecanumDriveEsp : public hardware_interface::SystemInterface
{
public:
  MecanumDriveEsp();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Config cfg_;
  EspComms esp_;
  
  // Four wheels for meccanum drive
  Wheel front_left_wheel_;
  Wheel front_right_wheel_;
  Wheel rear_left_wheel_;
  Wheel rear_right_wheel_;
  
  rclcpp::Logger logger_;
  std::chrono::time_point<std::chrono::system_clock> time_;
};

#endif // MECANUMDRIVE_ESP_REAL_ROBOT_H