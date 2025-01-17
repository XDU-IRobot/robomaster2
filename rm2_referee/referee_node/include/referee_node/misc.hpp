
#ifndef DECLARATIONS_HPP_
#define DECLARATIONS_HPP_

#include "rm2_referee_msgs/msg/custom_robot_data.hpp"
#include "rm2_referee_msgs/msg/remote_control.hpp"
#include "rm2_referee_msgs/msg/buff.hpp"
#include "rm2_referee_msgs/msg/dart_client_cmd.hpp"
#include "rm2_referee_msgs/msg/dart_info.hpp"
#include "rm2_referee_msgs/msg/event_data.hpp"
#include "rm2_referee_msgs/msg/game_result.hpp"
#include "rm2_referee_msgs/msg/game_robot_hp.hpp"
#include "rm2_referee_msgs/msg/game_status.hpp"
#include "rm2_referee_msgs/msg/ground_robot_position.hpp"
#include "rm2_referee_msgs/msg/hurt_data.hpp"
#include "rm2_referee_msgs/msg/power_heat_data.hpp"
#include "rm2_referee_msgs/msg/projectile_allowance.hpp"
#include "rm2_referee_msgs/msg/radar_info.hpp"
#include "rm2_referee_msgs/msg/radar_mark_data.hpp"
#include "rm2_referee_msgs/msg/referee_warning.hpp"
#include "rm2_referee_msgs/msg/rfid_status.hpp"
#include "rm2_referee_msgs/msg/robot_pos.hpp"
#include "rm2_referee_msgs/msg/robot_status.hpp"
#include "rm2_referee_msgs/msg/sentry_info.hpp"
#include "rm2_referee_msgs/msg/shoot_data.hpp"
#include "rm2_referee_msgs/msg/map_command.hpp"
#include "rm2_referee_msgs/srv/tx.hpp"

/**
 * @brief 所有裁判系统会发送的数据包和对应的命令码
 */
enum class OpCodeEnum : uint16_t {
  kGameStatus = 0x1,
  kGameResult = 0x2,
  kGameRobotHp = 0x3,
  kEventData = 0x101,
  kRefereeWarning = 0x104,
  kDartInformation = 0x105,
  kRobotStatus = 0x201,
  kPowerHeatData = 0x202,
  kRobotPos = 0x203,
  kBuff = 0x204,
  kHurtData = 0x206,
  kShootData = 0x207,
  kProjectileAllowance = 0x208,
  kRfidStatus = 0x209,
  kDartClientCmd = 0x20a,
  kGroundRobotPosition = 0x20b,
  kRadarMarkData = 0x20c,
  kSentryInfo = 0x20d,
  kRadarInfo = 0x20e,
  // kRobotInteractionData = 0x301,   // 数据方向为发送，不接收不需要解析
  // kSentryCmd = 0x120,              // 哨兵发
  // kRadarCmd = 0x121,               // 雷达发
  kCustomRobotData = 0x302,  // 图传链路
  kMapCommand = 0x303,
  kRemoteControl = 0x304,  // 图传链路
};

#endif
