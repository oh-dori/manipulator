#include "arduino_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arduino_hardware_interface
{
hardware_interface::CallbackReturn ArduinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // URDF로부터 파라미터 읽기
  const std::string serial_port = info_.hardware_parameters["serial_port"];
  const int baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  
  // 시리얼 드라이버 초기화
  serial_driver_ = std::make_unique<ArduinoSerialDriver>();
  if (!serial_driver_->open(serial_port, baud_rate))
  {
    RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Failed to open serial port %s", serial_port.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 상태 및 커맨드 벡터 초기화
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Successfully initialized!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Activating ...please wait...");
  // 초기 위치 설정 등
  RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Deactivating ...please wait...");
  serial_driver_->close();
  RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 서보모터는 피드백이 없으므로, 마지막으로 보낸 명령을 현재 상태로 간주 (Open-loop)
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    hw_states_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ros2_control로부터 받은 명령(라디안)을 아두이노로 전송 (0-180도 변환 필요)
  std::string msg = "";
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // 라디안을 각도로 변환 (예시: joint_1은 -90~90도, joint_2는 0~180도 등 URDF에 맞춰 변환)
    // 이 부분은 각 조인트의 특성에 맞게 정교한 변환 로직이 필요합니다.
    // 간단한 예시로 0~180도로 변환
    int angle = static_cast<int>((hw_commands_[i] + M_PI/2) * 180.0 / M_PI);
    angle = std::max(0, std::min(180, angle)); // 0-180 범위 제한
    msg += std::to_string(angle) + ",";
  }
  serial_driver_->write(msg + "\n");
  return hardware_interface::return_type::OK;
}
}  // namespace arduino_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arduino_hardware_interface::ArduinoHardwareInterface, hardware_interface::SystemInterface)