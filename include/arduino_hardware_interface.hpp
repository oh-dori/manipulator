#ifndef MANIPULATOR_ARDUINO_HARDWARE_INTERFACE_HPP
#define MANIPULATOR_ARDUINO_HARDWARE_INTERFACE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"
#include "arduino_serial_driver.hpp"

namespace arduino_hardware_interface
{
class ArduinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoHardwareInterface)

  MANIPULATOR_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  MANIPULATOR_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MANIPULATOR_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MANIPULATOR_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  MANIPULATOR_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  MANIPULATOR_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MANIPULATOR_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointMapping
  {
    double command_min;
    double command_max;
    double servo_min_angle;
    double servo_max_angle;
  };

  double command_to_servo_angle(std::size_t joint_index, double command) const;

  std::unique_ptr<ArduinoSerialDriver> serial_driver_;
  std::string serial_port_;
  int baud_rate_{115200};
  double write_rate_hz_{20.0};
  std::chrono::steady_clock::time_point last_write_time_{};
  std::vector<JointMapping> joint_mappings_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
};
}  // namespace arduino_hardware_interface
#endif // MANIPULATOR_ARDUINO_HARDWARE_INTERFACE_HPP
