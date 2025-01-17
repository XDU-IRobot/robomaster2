
#include "referee_node.hpp"

RefereeNode::RefereeNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("control_node", options),
      normal_decoder_(std::bind(&RefereeNode::PublishMsg, this, std::placeholders::_1, std::placeholders::_2)),
      fpv_decoder_(std::bind(&RefereeNode::PublishMsg, this, std::placeholders::_1, std::placeholders::_2)) {
  this->SpawnPublishers();
  this->GetParameters();

  // 初始化串口、启动线程
  if (this->param_enable_normal_) {
    RCLCPP_INFO(this->get_logger(), "Normal data link: Enabled @ %s", this->param_normal_tty_device_.c_str());
    this->SerialInit(this->param_normal_tty_device_, this->normal_serial_);
    this->normal_serial_rx_thread_ = std::thread([this] {
      for (;;) {
        this->normal_decoder_ << this->normal_serial_->read();
      }
    });
  } else {
    RCLCPP_INFO(this->get_logger(), "Normal data link: Disabled");
  }
  if (this->param_enable_fpv_) {
    RCLCPP_INFO(this->get_logger(), "FPV data link: Enabled @ %s", this->param_fpv_tty_device_.c_str());
    this->SerialInit(this->param_fpv_tty_device_, this->fpv_serial_);
    this->fpv_serial_rx_thread_ = std::thread([this] {
      for (;;) {
        this->fpv_decoder_ << this->fpv_serial_->read();
      }
    });
  } else {
    RCLCPP_INFO(this->get_logger(), "FPV data link: Disabled");
  }

  if (!this->param_enable_normal_ && !this->param_enable_fpv_) {
    RCLCPP_ERROR(this->get_logger(), "Both normal and FPV data links are disabled, exiting...");
    rclcpp::shutdown();
  }

  if (this->param_enable_normal_) {
    // 如果启用了常规链路，就创建裁判系统服务
    this->referee_srv_ = this->create_service<rm2_referee_msgs::srv::Tx>(
        "/rm2_referee/tx", [this](const std::shared_ptr<rm2_referee_msgs::srv::Tx::Request> request,
                                  std::shared_ptr<rm2_referee_msgs::srv::Tx::Response> response) {
          if (this->normal_serial_->write(std::string(request->data.begin(), request->data.end())) ==
              request->data.size()) {
            response->ok = true;
          } else {
            response->ok = false;
          }
          response->header.stamp = this->get_clock()->now();
        });
    RCLCPP_INFO(this->get_logger(), "Referee service is now at /rm2_referee/tx");
  }
}

RefereeNode::~RefereeNode() {
  if (this->param_enable_normal_) {
    this->normal_serial_->close();
  }
  if (this->param_enable_fpv_) {
    this->fpv_serial_->close();
  }
}

void RefereeNode::SpawnPublishers() {
  game_status_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::GameStatus>("/rm2_referee/game_status", rclcpp::SensorDataQoS());
  game_result_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::GameResult>("/rm2_referee/game_result", rclcpp::SensorDataQoS());
  game_robot_hp_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::GameRobotHP>("/rm2_referee/game_robot_hp", rclcpp::SensorDataQoS());
  event_data_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::EventData>("/rm2_referee/event_data", rclcpp::SensorDataQoS());
  referee_warning_pub_ = this->create_publisher<rm2_referee_msgs::msg::RefereeWarning>("/rm2_referee/referee_warning",
                                                                                       rclcpp::SensorDataQoS());
  dart_info_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::DartInfo>("/rm2_referee/dart_info", rclcpp::SensorDataQoS());
  robot_status_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::RobotStatus>("/rm2_referee/robot_status", rclcpp::SensorDataQoS());
  power_heat_data_pub_ = this->create_publisher<rm2_referee_msgs::msg::PowerHeatData>("/rm2_referee/power_heat_data",
                                                                                      rclcpp::SensorDataQoS());
  robot_pos_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::RobotPos>("/rm2_referee/robot_pos", rclcpp::SensorDataQoS());
  buff_pub_ = this->create_publisher<rm2_referee_msgs::msg::Buff>("/rm2_referee/buff", rclcpp::SensorDataQoS());
  hurt_data_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::HurtData>("/rm2_referee/hurt_data", rclcpp::SensorDataQoS());
  shoot_data_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::ShootData>("/rm2_referee/shoot_data", rclcpp::SensorDataQoS());
  projectile_allowance_pub_ = this->create_publisher<rm2_referee_msgs::msg::ProjectileAllowance>(
      "/rm2_referee/projectile_allowance", rclcpp::SensorDataQoS());
  rfid_status_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::RFIDStatus>("/rm2_referee/rfid_status", rclcpp::SensorDataQoS());
  dart_client_cmd_pub_ = this->create_publisher<rm2_referee_msgs::msg::DartClientCmd>("/rm2_referee/dart_client_cmd",
                                                                                      rclcpp::SensorDataQoS());
  ground_robot_position_pub_ = this->create_publisher<rm2_referee_msgs::msg::GroundRobotPosition>(
      "/rm2_referee/ground_robot_position", rclcpp::SensorDataQoS());
  radar_mark_data_pub_ = this->create_publisher<rm2_referee_msgs::msg::RadarMarkData>("/rm2_referee/radar_mark_data",
                                                                                      rclcpp::SensorDataQoS());
  sentry_info_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::SentryInfo>("/rm2_referee/sentry_info", rclcpp::SensorDataQoS());
  radar_info_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::RadarInfo>("/rm2_referee/radar_info", rclcpp::SensorDataQoS());
  custom_robot_data_pub_ = this->create_publisher<rm2_referee_msgs::msg::CustomRobotData>(
      "/rm2_referee/custom_robot_data", rclcpp::SensorDataQoS());
  map_command_pub_ =
      this->create_publisher<rm2_referee_msgs::msg::MapCommand>("/rm2_referee/map_command", rclcpp::SensorDataQoS());
  remote_control_pub_ = this->create_publisher<rm2_referee_msgs::msg::RemoteControl>("/rm2_referee/remote_control",
                                                                                     rclcpp::SensorDataQoS());
  RCLCPP_INFO(this->get_logger(), "Publishers spawned");
}

void RefereeNode::GetParameters() {
  this->get_parameter("enable_normal", param_enable_normal_);
  this->get_parameter("enable_fpv", param_enable_fpv_);
  this->get_parameter("normal_tty_device", param_normal_tty_device_);
  this->get_parameter("fpv_tty_device", param_fpv_tty_device_);
}

/**
 * @brief 初始化串口
 * @param tty_device 串口设备名
 * @param serial     Serial对象指针
 */
void RefereeNode::SerialInit(std::string tty_device, std::unique_ptr<serial::Serial> &serial) {
  serial = std::make_unique<serial::Serial>(tty_device, 115200, serial::Timeout::simpleTimeout(1000));
  if (!serial->isOpen()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open normal serial port: %s", tty_device.c_str());
    rclcpp::shutdown();
  }
  RCLCPP_INFO(this->get_logger(), "Serial port %s is open", tty_device.c_str());
}

/**
 * @brief 解包完成的回调函数，把消息发布出去
 * @param packet_payload  解包后的数据，不含帧头帧尾
 * @param op_code         数据包对应的操作码
 */
void RefereeNode::PublishMsg(uint8_t *packet_payload, OpCodeEnum op_code) {
  RCLCPP_DEBUG(this->get_logger(), "Received a packet with OpCode: %d", static_cast<int>(op_code));
  switch (op_code) {
    case OpCodeEnum::kGameStatus: {
      this->game_status_msg_.header.stamp = this->get_clock()->now();
      this->game_status_msg_.game_type = (packet_payload[0] >> 4);
      this->game_status_msg_.game_progress = packet_payload[0] & 0b00001111;
      this->game_status_msg_.stage_remain_time = (packet_payload[2] << 8) | packet_payload[1];
      memcpy(&this->game_status_msg_.sync_timestamp, &packet_payload[3], 8);
      this->game_status_pub_->publish(this->game_status_msg_);
      break;
    }
    case OpCodeEnum::kGameResult: {
      this->game_result_msg_.header.stamp = this->get_clock()->now();
      this->game_result_msg_.winner = packet_payload[0];
      this->game_result_pub_->publish(this->game_result_msg_);
      break;
    }
    case OpCodeEnum::kGameRobotHp: {
      this->game_robot_hp_msg_.header.stamp = this->get_clock()->now();
      this->game_robot_hp_msg_.red_1_robot_hp = (packet_payload[1] << 8) | packet_payload[0];
      this->game_robot_hp_msg_.red_2_robot_hp = (packet_payload[3] << 8) | packet_payload[2];
      this->game_robot_hp_msg_.red_3_robot_hp = (packet_payload[5] << 8) | packet_payload[4];
      this->game_robot_hp_msg_.red_4_robot_hp = (packet_payload[7] << 8) | packet_payload[6];
      this->game_robot_hp_msg_.reserved = (packet_payload[9] << 8) | packet_payload[8];
      this->game_robot_hp_msg_.red_7_robot_hp = (packet_payload[11] << 8) | packet_payload[10];
      this->game_robot_hp_msg_.red_outpost_hp = (packet_payload[13] << 8) | packet_payload[12];
      this->game_robot_hp_msg_.red_base_hp = (packet_payload[15] << 8) | packet_payload[14];
      this->game_robot_hp_msg_.blue_1_robot_hp = (packet_payload[17] << 8) | packet_payload[16];
      this->game_robot_hp_msg_.blue_2_robot_hp = (packet_payload[19] << 8) | packet_payload[18];
      this->game_robot_hp_msg_.blue_3_robot_hp = (packet_payload[21] << 8) | packet_payload[20];
      this->game_robot_hp_msg_.blue_4_robot_hp = (packet_payload[23] << 8) | packet_payload[22];
      this->game_robot_hp_msg_.reserved_2 = (packet_payload[25] << 8) | packet_payload[24];
      this->game_robot_hp_msg_.blue_7_robot_hp = (packet_payload[27] << 8) | packet_payload[26];
      this->game_robot_hp_msg_.blue_outpost_hp = (packet_payload[29] << 8) | packet_payload[28];
      this->game_robot_hp_msg_.blue_base_hp = (packet_payload[31] << 8) | packet_payload[30];
      this->game_robot_hp_pub_->publish(this->game_robot_hp_msg_);
      break;
    }
    case OpCodeEnum::kEventData: {
      this->event_data_msg_.header.stamp = this->get_clock()->now();
      memcpy(&this->event_data_msg_.event_data, packet_payload, 4);
      this->event_data_pub_->publish(this->event_data_msg_);
      break;
    }
    case OpCodeEnum::kRefereeWarning: {
      this->referee_warning_msg_.header.stamp = this->get_clock()->now();
      this->referee_warning_msg_.level = packet_payload[0];
      this->referee_warning_msg_.offending_robot_id = packet_payload[1];
      this->referee_warning_msg_.count = packet_payload[2];
      this->referee_warning_pub_->publish(this->referee_warning_msg_);
      break;
    }
    case OpCodeEnum::kDartInformation: {
      this->dart_info_msg_.header.stamp = this->get_clock()->now();
      this->dart_info_msg_.dart_remaining_time = packet_payload[0];
      this->dart_info_msg_.dart_info = (packet_payload[2] << 8) | packet_payload[1];
      this->dart_info_pub_->publish(this->dart_info_msg_);
      break;
    }
    case OpCodeEnum::kRobotStatus: {
      this->robot_status_msg_.header.stamp = this->get_clock()->now();
      this->robot_status_msg_.robot_id = packet_payload[0];
      this->robot_status_msg_.robot_level = packet_payload[1];
      this->robot_status_msg_.current_hp = (packet_payload[3] << 8) | packet_payload[2];
      this->robot_status_msg_.maximum_hp = (packet_payload[5] << 8) | packet_payload[4];
      this->robot_status_msg_.shooter_barrel_cooling_value = (packet_payload[7] << 8) | packet_payload[6];
      this->robot_status_msg_.shooter_barrel_heat_limit = (packet_payload[9] << 8) | packet_payload[8];
      this->robot_status_msg_.chassis_power_limit = (packet_payload[11] << 8) | packet_payload[10];
      this->robot_status_msg_.power_management_gimbal_output = packet_payload[12] & 0b00000001;
      this->robot_status_msg_.power_management_chassis_output = packet_payload[12] & 0b00000010;
      this->robot_status_msg_.power_management_shooter_output = packet_payload[12] & 0b00000100;
      this->robot_status_pub_->publish(this->robot_status_msg_);
      break;
    }
    case OpCodeEnum::kPowerHeatData: {
      this->power_heat_data_msg_.header.stamp = this->get_clock()->now();
      memcpy(&this->power_heat_data_msg_.reserved, &packet_payload[0], 2);
      memcpy(&this->power_heat_data_msg_.reserved_2, &packet_payload[2], 2);
      memcpy(&this->power_heat_data_msg_.reserved_3, &packet_payload[4], 4);
      memcpy(&this->power_heat_data_msg_.buffer_energy, &packet_payload[8], 2);
      memcpy(&this->power_heat_data_msg_.shooter_17mm_1_barrel_heat, &packet_payload[10], 2);
      memcpy(&this->power_heat_data_msg_.shooter_17mm_2_barrel_heat, &packet_payload[12], 2);
      memcpy(&this->power_heat_data_msg_.shooter_42mm_barrel_heat, &packet_payload[14], 2);
      this->power_heat_data_pub_->publish(this->power_heat_data_msg_);
      break;
    }
    case OpCodeEnum::kRobotPos: {
      this->robot_pos_msg_.header.stamp = this->get_clock()->now();
      memcpy(&this->robot_pos_msg_.x, &packet_payload[0], 4);
      memcpy(&this->robot_pos_msg_.y, &packet_payload[4], 4);
      memcpy(&this->robot_pos_msg_.angle, &packet_payload[8], 4);
      this->robot_pos_pub_->publish(this->robot_pos_msg_);
      break;
    }
    case OpCodeEnum::kBuff: {
      this->buff_msg_.header.stamp = this->get_clock()->now();
      this->buff_msg_.recovery_buff = packet_payload[0];
      this->buff_msg_.cooling_buff = packet_payload[1];
      this->buff_msg_.defence_buff = packet_payload[2];
      this->buff_msg_.vulnerability_buff = packet_payload[3];
      this->buff_msg_.attack_buff = (packet_payload[4] << 8) | packet_payload[5];
      this->buff_pub_->publish(this->buff_msg_);
      break;
    }
    case OpCodeEnum::kHurtData: {
      this->hurt_data_msg_.header.stamp = this->get_clock()->now();
      this->hurt_data_msg_.armor_id = (packet_payload[0] & 0b00001111);
      this->hurt_data_msg_.hp_deduction_reason = (packet_payload[0] >> 4);
      this->hurt_data_pub_->publish(this->hurt_data_msg_);
      break;
    }
    case OpCodeEnum::kShootData: {
      this->shoot_data_msg_.header.stamp = this->get_clock()->now();
      this->shoot_data_msg_.bullet_type = packet_payload[0];
      this->shoot_data_msg_.shooter_number = packet_payload[1];
      this->shoot_data_msg_.launching_frequency = packet_payload[2];
      memcpy(&this->shoot_data_msg_.initial_speed, &packet_payload[3], 4);
      this->shoot_data_pub_->publish(this->shoot_data_msg_);
      break;
    }
    case OpCodeEnum::kProjectileAllowance: {
      this->projectile_allowance_msg_.header.stamp = this->get_clock()->now();
      memcpy(&this->projectile_allowance_msg_.projectile_allowance_17mm, &packet_payload[0], 2);
      memcpy(&this->projectile_allowance_msg_.projectile_allowance_42mm, &packet_payload[2], 2);
      memcpy(&this->projectile_allowance_msg_.remaining_gold_coin, &packet_payload[4], 2);
      this->projectile_allowance_pub_->publish(this->projectile_allowance_msg_);
      break;
    }
    case OpCodeEnum::kRfidStatus: {
      this->rfid_status_msg_.header.stamp = this->get_clock()->now();
      memcpy(&this->rfid_status_msg_.rfid_status, packet_payload, 4);
      this->rfid_status_pub_->publish(this->rfid_status_msg_);
      break;
    }
    case OpCodeEnum::kDartClientCmd: {
      this->dart_client_cmd_msg_.header.stamp = this->get_clock()->now();
      this->dart_client_cmd_msg_.dart_launch_opening_status = packet_payload[0];
      this->dart_client_cmd_msg_.reserved = packet_payload[1];
      this->dart_client_cmd_msg_.target_change_time = (packet_payload[3] << 8) | packet_payload[2];
      this->dart_client_cmd_msg_.latest_launch_cmd_time = (packet_payload[5] << 8) | packet_payload[4];
      this->dart_client_cmd_pub_->publish(this->dart_client_cmd_msg_);
      break;
    }
    case OpCodeEnum::kGroundRobotPosition: {
      this->ground_robot_position_msg_.header.stamp = this->get_clock()->now();
      memcpy(&this->ground_robot_position_msg_.hero_x, &packet_payload[0], 4);
      memcpy(&this->ground_robot_position_msg_.hero_y, &packet_payload[4], 4);
      memcpy(&this->ground_robot_position_msg_.engineer_x, &packet_payload[8], 4);
      memcpy(&this->ground_robot_position_msg_.engineer_y, &packet_payload[12], 4);
      memcpy(&this->ground_robot_position_msg_.standard_3_x, &packet_payload[16], 4);
      memcpy(&this->ground_robot_position_msg_.standard_3_y, &packet_payload[20], 4);
      memcpy(&this->ground_robot_position_msg_.standard_4_x, &packet_payload[24], 4);
      memcpy(&this->ground_robot_position_msg_.standard_4_y, &packet_payload[28], 4);
      memcpy(&this->ground_robot_position_msg_.reserved, &packet_payload[32], 4);
      memcpy(&this->ground_robot_position_msg_.reserved_2, &packet_payload[36], 4);
      this->ground_robot_position_pub_->publish(this->ground_robot_position_msg_);
      break;
    }
    case OpCodeEnum::kRadarMarkData: {
      this->radar_mark_data_msg_.header.stamp = this->get_clock()->now();
      this->radar_mark_data_msg_.mark_progress = packet_payload[0];
      break;
    }
    case OpCodeEnum::kSentryInfo: {
      this->sentry_info_msg_.header.stamp = this->get_clock()->now();
      memcpy(&sentry_info_msg_.sentry_info, packet_payload, 4);
      this->sentry_info_pub_->publish(this->sentry_info_msg_);
      break;
    }
    case OpCodeEnum::kRadarInfo: {
      this->radar_info_msg_.header.stamp = this->get_clock()->now();
      this->radar_info_msg_.radar_info = packet_payload[0];
      this->radar_info_pub_->publish(this->radar_info_msg_);
      break;
    }
    case OpCodeEnum::kCustomRobotData: {
      this->custom_robot_data_msg_.header.stamp = this->get_clock()->now();
      memcpy(&this->custom_robot_data_msg_.data, packet_payload, 30);
      this->custom_robot_data_pub_->publish(this->custom_robot_data_msg_);
      break;
    }
    case OpCodeEnum::kMapCommand: {
      this->map_command_msg_.header.stamp = this->get_clock()->now();
      memcpy(&this->map_command_msg_.target_position_x, packet_payload, 4);
      memcpy(&this->map_command_msg_.target_position_y, &packet_payload[4], 4);
      this->map_command_msg_.cmd_keyboard = packet_payload[8];
      this->map_command_msg_.target_robot_id = packet_payload[9];
      this->map_command_msg_.cmd_source = packet_payload[10];
      this->map_command_pub_->publish(this->map_command_msg_);
      break;
    }
    case OpCodeEnum::kRemoteControl: {
      this->remote_control_msg_.header.stamp = this->get_clock()->now();
      this->remote_control_msg_.mouse_x = (packet_payload[1] << 8) | packet_payload[0];
      this->remote_control_msg_.mouse_y = (packet_payload[3] << 8) | packet_payload[2];
      this->remote_control_msg_.mouse_z = (packet_payload[5] << 8) | packet_payload[4];
      this->remote_control_msg_.left_button_down = packet_payload[6];
      this->remote_control_msg_.right_button_down = packet_payload[7];
      this->remote_control_msg_.keyboard_value = (packet_payload[9] << 8) | packet_payload[8];
      this->remote_control_msg_.reserved = (packet_payload[11] << 8) | packet_payload[10];
      this->remote_control_pub_->publish(this->remote_control_msg_);
      break;
    }
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RefereeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
