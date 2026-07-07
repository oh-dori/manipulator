#include "arduino_serial_driver.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace arduino_hardware_interface
{
namespace
{

bool to_termios_baud_rate(int baud_rate, speed_t & speed)
{
  switch (baud_rate) {
    case 9600:
      speed = B9600;
      return true;
    case 19200:
      speed = B19200;
      return true;
    case 38400:
      speed = B38400;
      return true;
    case 57600:
      speed = B57600;
      return true;
    case 115200:
      speed = B115200;
      return true;
    default:
      return false;
  }
}

}  // namespace

ArduinoSerialDriver::~ArduinoSerialDriver()
{
  close();
}

bool ArduinoSerialDriver::open(const std::string & port, int baud_rate)
{
  close();

  speed_t speed{};
  if (!to_termios_baud_rate(baud_rate, speed)) {
    last_error_ = "unsupported baud rate: " + std::to_string(baud_rate);
    return false;
  }

  file_descriptor_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (file_descriptor_ < 0) {
    last_error_ = "failed to open " + port + ": " + std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(file_descriptor_, &tty) != 0) {
    last_error_ = "tcgetattr failed: " + std::string(std::strerror(errno));
    close();
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(file_descriptor_, TCSANOW, &tty) != 0) {
    last_error_ = "tcsetattr failed: " + std::string(std::strerror(errno));
    close();
    return false;
  }

  tcflush(file_descriptor_, TCIOFLUSH);
  last_error_.clear();
  return true;
}

bool ArduinoSerialDriver::write(const std::string & data)
{
  if (!is_open()) {
    last_error_ = "serial port is not open";
    return false;
  }

  std::size_t written = 0;
  while (written < data.size()) {
    const auto result = ::write(
      file_descriptor_, data.data() + written, data.size() - written);
    if (result > 0) {
      written += static_cast<std::size_t>(result);
      continue;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }

    last_error_ = "serial write failed: " + std::string(std::strerror(errno));
    return false;
  }

  return true;
}

void ArduinoSerialDriver::close()
{
  if (file_descriptor_ >= 0) {
    ::close(file_descriptor_);
    file_descriptor_ = -1;
  }
}

bool ArduinoSerialDriver::is_open() const
{
  return file_descriptor_ >= 0;
}

const std::string & ArduinoSerialDriver::last_error() const
{
  return last_error_;
}

}  // namespace arduino_hardware_interface
