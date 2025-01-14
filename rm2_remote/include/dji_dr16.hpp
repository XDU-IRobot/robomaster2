
#ifndef DR16_NODE_HPP_
#define DR16_NODE_HPP_

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

struct DR16 {
  int axes[5]{0};           // [0]: right_x, [1]: right_y, [2]: left_x, [3]: left_y, [4]: dial
  int mouse[3]{0};          // [0]: x, [1]: y, [2]: z
  bool mouse_button[2]{0};  // [0]: left, [1]: right
  int switches[2]{0};       // [0]: right, [1]: left
  int keyboard_key{};       // 每一位代表一个键，0为未按下，1为按下
};

class Dr16Node : public rclcpp::Node {
 public:
  Dr16Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~Dr16Node();

 private:
  void SerialRxThread();

  void SerialInit();
  void SerialRead();

  void Unpack(int received_total_data_len);

 private:
  std::string param_serial_port_{};
  sensor_msgs::msg::Joy joy_msg_;
  std::thread serial_rx_thread_{};
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_{nullptr};
  DR16 dr16_;

  // Serial port
  int serial_fd_{};
  uint8_t serial_buffer_[100]{};
};

#endif  // DR16_NODE_HPP_