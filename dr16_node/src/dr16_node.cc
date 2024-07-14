
#include "dr16_node.hpp"

#include <iostream>
#include <chrono>

#include <fcntl.h> /* File control definitions */
#include <cstring>
#include <unistd.h>

#define termios asmtermios
#include <asm/termios.h>
#undef termios

#include <termios.h>

extern "C" {
extern int ioctl(int __fd, unsigned long int __request, ...) throw();
}

using namespace std::chrono_literals;

Dr16Node::Dr16Node(const rclcpp::NodeOptions &options) : rclcpp::Node("dr16_node", options) {
  this->declare_parameter<std::string>("tty_device", "/dev/ttyTHS0");
  this->get_parameter<std::string>("tty_device", this->param_tty_device_);

  RCLCPP_INFO(this->get_logger(), "tty_device: %s", this->param_tty_device_.c_str());

  this->joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/rmos_ec_referee/joy", 10);
  this->SerialInit();

  this->serial_rx_thread_ = std::thread([this]() {
    while (rclcpp::ok()) {
      this->SerialRead();
    }
  });
}

Dr16Node::~Dr16Node() { ::close(serial_fd_); }

void Dr16Node::SerialInit() {
  this->serial_fd_ = ::open(this->param_tty_device_.c_str(), O_RDWR | O_NOCTTY);
  if (this->serial_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", std::strerror(errno));
    rclcpp::shutdown();
  }

  struct termios2 options {};
  ::ioctl(this->serial_fd_, TCGETS2, &options);

  options.c_cflag &= ~CBAUD;
  options.c_cflag |= BOTHER;

  options.c_cflag |= PARENB;   // enable parity
  options.c_cflag &= ~PARODD;  // even parity
  options.c_cflag &= ~CSTOPB;  // 1 stop bit
  options.c_cflag &= ~CSIZE;   // mask the character size bits
  options.c_cflag |= CS8;      // 8 data bits

  options.c_ispeed = 100000;
  options.c_ospeed = 100000;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_iflag &= ~IGNBRK;  // disable break processing

  options.c_lflag = 0;
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 0;

  options.c_oflag = 0;                  // no remapping, no delays
  options.c_cflag |= (CLOCAL | CREAD);  // ignore modem controls, enable reading
  ::ioctl(this->serial_fd_, TCSETS2, &options);

  tcflush(this->serial_fd_, TCIOFLUSH);  // flush the buffer
}

void Dr16Node::SerialRead() {
  int timeout = 0;  // time out of one package
  int count = 0;    // count of bit of one package
  while (timeout < 3) {
    // Read a byte //
    size_t n = ::read(this->serial_fd_, &this->serial_buffer_[count], sizeof(uint8_t));
    if (n == 0) {
      ++timeout;
      std::this_thread::sleep_for(1ms);
    } else if (n == 1) {
      timeout = 0;
      ++count;
    }
    if (count >= 100) {
      RCLCPP_ERROR(
          this->get_logger(),
          "DR16 node has continuously received 100 bytes of data without a 14ms break, this should not happen!");
      return;
    }
  }
  if (count >= 18) {
    this->Unpack(count);
  }
}

void Dr16Node::Unpack(int received_total_data_len) {
  uint8_t *buf = &this->serial_buffer_[received_total_data_len - 18];
  this->dr16_.axes[0] = (buf[0] | (buf[1] << 8)) & 0x07ff;         //!< Channel 0
  this->dr16_.axes[1] = ((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff;  //!< Channel 1
  this->dr16_.axes[2] = ((buf[2] >> 6) | (buf[3] << 2) |           //!< Channel 2
                         (buf[4] << 10)) &
                        0x07ff;
  this->dr16_.axes[3] = ((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff;  //!< Channel 3
  this->dr16_.switches[0] = ((buf[5] >> 4) & 0x0003);              //!< Switch left
  this->dr16_.switches[1] = (((buf[5] >> 4) & 0x000c) >> 2);       //!< Switch right
  this->dr16_.mouse[0] = buf[6] | (buf[7] << 8);                   //!< Mouse X axis
  this->dr16_.mouse[1] = buf[8] | (buf[9] << 8);                   //!< Mouse Y axis
  this->dr16_.mouse[2] = buf[10] | (buf[11] << 8);                 //!< Mouse Z axis
  this->dr16_.mouse_button[0] = buf[12];                           //!< Mouse Left Is Press ?
  this->dr16_.mouse_button[1] = buf[13];                           //!< Mouse Right Is Press ?
  this->dr16_.keyboard_key = buf[14] | (buf[15] << 8);             //!< KeyBoard value
  this->dr16_.axes[4] = buf[16] | (buf[17] << 8);                  // NULL

  this->dr16_.axes[0] -= 1024;
  this->dr16_.axes[1] -= 1024;
  this->dr16_.axes[2] -= 1024;
  this->dr16_.axes[3] -= 1024;
  this->dr16_.axes[4] -= 1024;

  this->joy_msg_.header.stamp = this->now();
  this->joy_msg_.axes = {(float)this->dr16_.axes[0] / 660.0f,  (float)this->dr16_.axes[1] / 660.0f,
                         (float)this->dr16_.axes[2] / 660.0f,  (float)this->dr16_.axes[3] / 660.0f,
                         (float)this->dr16_.axes[4] / 660.0f,  (float)this->dr16_.mouse[0] / 660.0f,
                         (float)this->dr16_.mouse[1] / 660.0f, (float)this->dr16_.mouse[2] / 660.0f};
  this->joy_msg_.buttons = {this->dr16_.switches[0], this->dr16_.switches[1], this->dr16_.mouse_button[0],
                            this->dr16_.mouse_button[1], this->dr16_.keyboard_key};
  this->joy_pub_->publish(this->joy_msg_);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dr16Node>());
  rclcpp::shutdown();
  return 0;
}
