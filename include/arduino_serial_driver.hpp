#ifndef MANIPULATOR__ARDUINO_SERIAL_DRIVER_HPP_
#define MANIPULATOR__ARDUINO_SERIAL_DRIVER_HPP_

#include <string>

namespace arduino_hardware_interface
{

class ArduinoSerialDriver
{
public:
  ArduinoSerialDriver() = default;
  ~ArduinoSerialDriver();

  ArduinoSerialDriver(const ArduinoSerialDriver &) = delete;
  ArduinoSerialDriver & operator=(const ArduinoSerialDriver &) = delete;

  bool open(const std::string & port, int baud_rate);
  bool write(const std::string & data);
  void close();
  bool is_open() const;
  const std::string & last_error() const;

private:
  int file_descriptor_{-1};
  std::string last_error_;
};

}  // namespace arduino_hardware_interface

#endif  // MANIPULATOR__ARDUINO_SERIAL_DRIVER_HPP_
