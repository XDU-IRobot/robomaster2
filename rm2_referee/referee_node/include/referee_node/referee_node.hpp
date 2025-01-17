
#ifndef REFEREE_NODE_HPP_
#define REFEREE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <array>
#include <thread>
#include <vector>

#include "misc.hpp"
#include "decoder.hpp"

#include "serial/serial.h"

class RefereeNode : public rclcpp::Node {
 public:
  explicit RefereeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
                                                                .allow_undeclared_parameters(true)
                                                                .automatically_declare_parameters_from_overrides(true));
  ~RefereeNode() override;

 private:
  void SpawnPublishers();
  void GetParameters();
  void SerialInit(std::string tty_device, std::unique_ptr<serial::Serial>& serial);
  void PublishMsg(uint8_t* data, OpCodeEnum op_code);

 private:
  /** <Normal members> **/
  std::unique_ptr<serial::Serial> normal_serial_{nullptr};
  std::unique_ptr<serial::Serial> fpv_serial_{nullptr};
  /** </Normal members> **/

 private:
  /** <Parameters> **/
  std::string param_normal_tty_device_{};
  std::string param_fpv_tty_device_{};
  bool param_enable_normal_{};
  bool param_enable_fpv_{};
  /** </Parameters> **/

 private:
  /** <Threads> **/
  std::thread normal_serial_rx_thread_;
  std::thread fpv_serial_rx_thread_;
  /** </Threads> **/
  /** <Decoders> **/
  RefereeDecoder normal_decoder_;
  RefereeDecoder fpv_decoder_;
  /** </Decoders> **/

 private:
  /** <Publishers> **/
  rclcpp::Publisher<rm2_referee_msgs::msg::GameStatus>::SharedPtr game_status_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::GameResult>::SharedPtr game_result_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::GameRobotHP>::SharedPtr game_robot_hp_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::EventData>::SharedPtr event_data_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::RefereeWarning>::SharedPtr referee_warning_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::DartInfo>::SharedPtr dart_info_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::PowerHeatData>::SharedPtr power_heat_data_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::RobotPos>::SharedPtr robot_pos_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::Buff>::SharedPtr buff_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::HurtData>::SharedPtr hurt_data_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::ShootData>::SharedPtr shoot_data_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::ProjectileAllowance>::SharedPtr projectile_allowance_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::RFIDStatus>::SharedPtr rfid_status_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::DartClientCmd>::SharedPtr dart_client_cmd_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::GroundRobotPosition>::SharedPtr ground_robot_position_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::RadarMarkData>::SharedPtr radar_mark_data_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::SentryInfo>::SharedPtr sentry_info_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::RadarInfo>::SharedPtr radar_info_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::CustomRobotData>::SharedPtr custom_robot_data_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::MapCommand>::SharedPtr map_command_pub_{nullptr};
  rclcpp::Publisher<rm2_referee_msgs::msg::RemoteControl>::SharedPtr remote_control_pub_{nullptr};
  /** </Publishers> **/

  /** <msgs> **/
  rm2_referee_msgs::msg::GameStatus game_status_msg_{};
  rm2_referee_msgs::msg::GameResult game_result_msg_{};
  rm2_referee_msgs::msg::GameRobotHP game_robot_hp_msg_{};
  rm2_referee_msgs::msg::EventData event_data_msg_{};
  rm2_referee_msgs::msg::RefereeWarning referee_warning_msg_{};
  rm2_referee_msgs::msg::DartInfo dart_info_msg_{};
  rm2_referee_msgs::msg::RobotStatus robot_status_msg_{};
  rm2_referee_msgs::msg::PowerHeatData power_heat_data_msg_{};
  rm2_referee_msgs::msg::RobotPos robot_pos_msg_{};
  rm2_referee_msgs::msg::Buff buff_msg_{};
  rm2_referee_msgs::msg::HurtData hurt_data_msg_{};
  rm2_referee_msgs::msg::ShootData shoot_data_msg_{};
  rm2_referee_msgs::msg::ProjectileAllowance projectile_allowance_msg_{};
  rm2_referee_msgs::msg::RFIDStatus rfid_status_msg_{};
  rm2_referee_msgs::msg::DartClientCmd dart_client_cmd_msg_{};
  rm2_referee_msgs::msg::GroundRobotPosition ground_robot_position_msg_{};
  rm2_referee_msgs::msg::RadarMarkData radar_mark_data_msg_{};
  rm2_referee_msgs::msg::SentryInfo sentry_info_msg_{};
  rm2_referee_msgs::msg::RadarInfo radar_info_msg_{};
  rm2_referee_msgs::msg::CustomRobotData custom_robot_data_msg_{};
  rm2_referee_msgs::msg::MapCommand map_command_msg_{};
  rm2_referee_msgs::msg::RemoteControl remote_control_msg_{};
  /** </msgs> **/

 private:
  rclcpp::Service<rm2_referee_msgs::srv::Tx>::SharedPtr referee_srv_{nullptr};
};

#endif
