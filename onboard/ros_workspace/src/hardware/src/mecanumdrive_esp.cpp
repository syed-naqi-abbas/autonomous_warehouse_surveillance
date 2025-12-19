#include "hardware/mecanumdrive_esp.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

MecanumDriveEsp::MecanumDriveEsp()
    : logger_(rclcpp::get_logger("MecanumDriveEsp"))
{}

hardware_interface::CallbackReturn MecanumDriveEsp::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Initializing...");

  // Load parameters from the URDF/XACRO
  // Using the updated member names as defined in the Config struct
  cfg_.left_front_wheel_name = info_.hardware_parameters["left_front_wheel_name"];
  cfg_.right_front_wheel_name = info_.hardware_parameters["right_front_wheīel_name"];
  cfg_.left_rear_wheel_name = info_.hardware_parameters["left_rear_wheel_name"];
  cfg_.right_rear_wheel_name = info_.hardware_parameters["right_rear_wheel_name"];
  
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  
  // Load individual encoder calibration parameters
  cfg_.left_front_enc_counts_per_rev = std::stoi(info_.hardware_parameters["left_front_enc_counts_per_rev"]);
  cfg_.right_front_enc_counts_per_rev = std::stoi(info_.hardware_parameters["right_front_enc_counts_per_rev"]);
  cfg_.left_rear_enc_counts_per_rev = std::stoi(info_.hardware_parameters["left_rear_enc_counts_per_rev"]);
  cfg_.right_rear_enc_counts_per_rev = std::stoi(info_.hardware_parameters["right_rear_enc_counts_per_rev"]);

  // Set up the wheels with their individual calibration values
  front_left_wheel_.setup(cfg_.left_front_wheel_name, cfg_.left_front_enc_counts_per_rev);
  front_right_wheel_.setup(cfg_.right_front_wheel_name, cfg_.right_front_enc_counts_per_rev);
  rear_left_wheel_.setup(cfg_.left_rear_wheel_name, cfg_.left_rear_enc_counts_per_rev);
  rear_right_wheel_.setup(cfg_.right_rear_wheel_name, cfg_.right_rear_enc_counts_per_rev);

  // Set up the Esp connection
  esp_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

  RCLCPP_INFO(logger_, "Finished Initialization");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumDriveEsp::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each of the 4 wheels
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Front Left
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    front_left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_left_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    front_left_wheel_.name, hardware_interface::HW_IF_POSITION, &front_left_wheel_.pos));

  // Front Right
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    front_right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_right_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    front_right_wheel_.name, hardware_interface::HW_IF_POSITION, &front_right_wheel_.pos));

  // Rear Left
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rear_left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rear_left_wheel_.name, hardware_interface::HW_IF_POSITION, &rear_left_wheel_.pos));

  // Rear Right
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rear_right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rear_right_wheel_.name, hardware_interface::HW_IF_POSITION, &rear_right_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumDriveEsp::export_command_interfaces()
{
  // We need to set up a velocity command interface for each of the 4 wheels
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    front_left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_left_wheel_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    front_right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_right_wheel_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    rear_left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    rear_right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn MecanumDriveEsp::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  esp_.sendEmptyMsg();
  
  // Assuming the Esp sketch expects PID values (Kp, Ki, Kd, Ko)
  // You might need to adjust this if your 4-wheel sketch needs different PID handling
  esp_.setPidValues(30, 0, 20, 100);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumDriveEsp::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumDriveEsp::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!esp_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // Assuming readEncoderValues now accepts 4 references for the 4 wheels
  esp_.readEncoderValues(
    front_left_wheel_.enc, 
    front_right_wheel_.enc,
    rear_left_wheel_.enc,
    rear_right_wheel_.enc
  );

  double delta_seconds = period.seconds();

  // Helper lambda to update wheel state
  auto update_wheel = [&](Wheel &wheel) {
      double pos_prev = wheel.pos;
      wheel.pos = wheel.calcEncAngle();
      wheel.vel = (wheel.pos - pos_prev) / delta_seconds;
  };

  update_wheel(front_left_wheel_);
  update_wheel(front_right_wheel_);
  update_wheel(rear_left_wheel_);
  update_wheel(rear_right_wheel_);

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type MecanumDriveEsp::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  if (!esp_.connected())
  {
    RCLCPP_ERROR_THROTTLE(
      logger_,
      steady_clock,
      2000,
      "⚠ Esp not connected!"
    );
    return hardware_interface::return_type::ERROR;
  }

  

  RCLCPP_INFO_THROTTLE(
      logger_,
      steady_clock,
      10,
      "Wheel cmds: FL=%.3f FR=%.3f RL=%.3f RR=%.3f  enc cmds: FL=%d FR=%d RL=%d RR=%d",
      front_left_wheel_.cmd,
      front_right_wheel_.cmd,
      rear_left_wheel_.cmd,
      rear_right_wheel_.cmd,
      front_left_wheel_.enc,
      front_right_wheel_.enc,
      rear_left_wheel_.enc,
      rear_right_wheel_.enc
  );

  

  int fl_motor = static_cast<int>(front_left_wheel_.cmd / front_left_wheel_.rads_per_count / cfg_.loop_rate);
  int fr_motor = static_cast<int>(front_right_wheel_.cmd / front_right_wheel_.rads_per_count / cfg_.loop_rate);
  int rl_motor = static_cast<int>(rear_left_wheel_.cmd / rear_left_wheel_.rads_per_count / cfg_.loop_rate);
  int rr_motor = static_cast<int>(rear_right_wheel_.cmd / rear_right_wheel_.rads_per_count / cfg_.loop_rate);

  esp_.setMotorValues(fl_motor, fr_motor, rl_motor, rr_motor);

  return hardware_interface::return_type::OK;
}



PLUGINLIB_EXPORT_CLASS(
  MecanumDriveEsp,
  hardware_interface::SystemInterface
)