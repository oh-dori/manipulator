#include "arduino_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <cctype>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arduino_hardware_interface
{
namespace
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("ArduinoHardwareInterface");

bool parse_double(
  const std::string & value, const std::string & parameter_name, double & result)
{
  try {
    std::size_t parsed_length = 0;
    result = std::stod(value, &parsed_length);
    if (parsed_length != value.size() || !std::isfinite(result)) {
      throw std::invalid_argument("not a finite number");
    }
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      LOGGER, "Invalid value '%s' for parameter '%s': %s",
      value.c_str(), parameter_name.c_str(), error.what());
    return false;
  }
}

bool parse_bool(
  const std::string & value, const std::string & parameter_name, bool & result)
{
  std::string normalized;
  normalized.reserve(value.size());
  for (const char character : value) {
    normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
  }

  if (normalized == "true" || normalized == "1") {
    result = true;
    return true;
  }
  if (normalized == "false" || normalized == "0") {
    result = false;
    return true;
  }

  RCLCPP_ERROR(
    LOGGER, "Invalid value '%s' for parameter '%s': expected true/false or 1/0",
    value.c_str(), parameter_name.c_str());
  return false;
}

}  // namespace

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto serial_port_it = info_.hardware_parameters.find("serial_port");
  const auto baud_rate_it = info_.hardware_parameters.find("baud_rate");
  if (
    serial_port_it == info_.hardware_parameters.end() ||
    baud_rate_it == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(LOGGER, "serial_port and baud_rate hardware parameters are required");
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = serial_port_it->second;
  try {
    baud_rate_ = std::stoi(baud_rate_it->second);
    const auto write_rate_it = info_.hardware_parameters.find("write_rate_hz");
    if (write_rate_it != info_.hardware_parameters.end()) {
      write_rate_hz_ = std::stod(write_rate_it->second);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(LOGGER, "Invalid hardware parameter: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto dry_run_it = info_.hardware_parameters.find("dry_run");
  if (
    dry_run_it != info_.hardware_parameters.end() &&
    !parse_bool(dry_run_it->second, "dry_run", dry_run_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (write_rate_hz_ <= 0.0 || !std::isfinite(write_rate_hz_)) {
    RCLCPP_ERROR(LOGGER, "write_rate_hz must be a finite positive number");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t joint_count = info_.joints.size();
  hw_commands_.resize(joint_count, 0.0);
  hw_positions_.resize(joint_count, 0.0);
  joint_mappings_.reserve(joint_count);

  for (std::size_t index = 0; index < joint_count; ++index) {
    const auto & joint = info_.joints[index];
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        LOGGER, "Joint '%s' must expose exactly one position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (
      joint.state_interfaces.size() != 1 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        LOGGER, "Joint '%s' must expose exactly one position state interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    double command_min = 0.0;
    double command_max = 0.0;
    double servo_min = 0.0;
    double servo_max = 180.0;

    const auto & command_interface = joint.command_interfaces[0];
    if (
      !parse_double(command_interface.min, joint.name + ".command.min", command_min) ||
      !parse_double(command_interface.max, joint.name + ".command.max", command_max))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto servo_min_it = joint.parameters.find("servo_min_angle");
    const auto servo_max_it = joint.parameters.find("servo_max_angle");
    if (
      servo_min_it == joint.parameters.end() ||
      servo_max_it == joint.parameters.end() ||
      !parse_double(servo_min_it->second, joint.name + ".servo_min_angle", servo_min) ||
      !parse_double(servo_max_it->second, joint.name + ".servo_max_angle", servo_max))
    {
      RCLCPP_ERROR(
        LOGGER, "Joint '%s' requires servo_min_angle and servo_max_angle parameters",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (command_min >= command_max) {
      RCLCPP_ERROR(LOGGER, "Joint '%s' has an invalid command range", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    double initial_position = 0.0;
    const auto initial_value_it = joint.state_interfaces[0].initial_value;
    if (
      !initial_value_it.empty() &&
      !parse_double(initial_value_it, joint.name + ".initial_value", initial_position))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    initial_position = std::clamp(initial_position, command_min, command_max);
    hw_commands_[index] = initial_position;
    hw_positions_[index] = initial_position;
    joint_mappings_.push_back({command_min, command_max, servo_min, servo_max});
  }

  serial_driver_ = std::make_unique<ArduinoSerialDriver>();
  RCLCPP_INFO(
    LOGGER, "Configured %zu joints for %s at %d baud (write rate %.1f Hz, dry_run: %s)",
    joint_count, serial_port_.c_str(), baud_rate_, write_rate_hz_, dry_run_ ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArduinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &hw_positions_[index]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArduinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    command_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &hw_commands_[index]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  hw_commands_ = hw_positions_;
  last_write_time_ = {};
  last_command_message_.clear();

  if (dry_run_) {
    RCLCPP_WARN(
      LOGGER,
      "Arduino hardware interface activated in dry-run mode; serial port will not be opened");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!serial_driver_->open(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(LOGGER, "Failed to open serial port: %s", serial_driver_->last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "Arduino hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (!dry_run_) {
    serial_driver_->close();
  }
  RCLCPP_INFO(LOGGER, "Arduino hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // The hobby servos do not return position feedback. Report the last accepted command.
  hw_positions_ = hw_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto now = std::chrono::steady_clock::now();
  const auto minimum_period = std::chrono::duration<double>(1.0 / write_rate_hz_);
  if (
    last_write_time_ != std::chrono::steady_clock::time_point{} &&
    now - last_write_time_ < minimum_period)
  {
    return hardware_interface::return_type::OK;
  }

  std::ostringstream message;
  message << std::fixed << std::setprecision(0);
  for (std::size_t index = 0; index < hw_commands_.size(); ++index) {
    if (!std::isfinite(hw_commands_[index])) {
      RCLCPP_ERROR(LOGGER, "Joint '%s' received a non-finite command", info_.joints[index].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    if (index > 0) {
      message << ',';
    }
    message << command_to_servo_angle(index, hw_commands_[index]);
  }
  // ROS 2 controller가 이미 계산한 중간 목표값이므로 Arduino에서 즉시 적용합니다.
  message << ",1\n";

  if (dry_run_) {
    const auto command_message = message.str();
    if (command_message != last_command_message_) {
      RCLCPP_INFO(LOGGER, "[dry-run] Arduino CSV: %s", command_message.c_str());
      last_command_message_ = command_message;
    }
    last_write_time_ = now;
    return hardware_interface::return_type::OK;
  }

  if (!serial_driver_->write(message.str())) {
    RCLCPP_ERROR(LOGGER, "Serial write failed: %s", serial_driver_->last_error().c_str());
    return hardware_interface::return_type::ERROR;
  }

  last_write_time_ = now;
  return hardware_interface::return_type::OK;
}

double ArduinoHardwareInterface::command_to_servo_angle(
  std::size_t joint_index, double command) const
{
  const auto & mapping = joint_mappings_[joint_index];
  const double clamped_command = std::clamp(
    command, mapping.command_min, mapping.command_max);
  const double ratio =
    (clamped_command - mapping.command_min) /
    (mapping.command_max - mapping.command_min);
  return mapping.servo_min_angle +
         ratio * (mapping.servo_max_angle - mapping.servo_min_angle);
}

}  // namespace arduino_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  arduino_hardware_interface::ArduinoHardwareInterface,
  hardware_interface::SystemInterface)
