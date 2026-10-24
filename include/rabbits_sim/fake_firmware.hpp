// 機体に依らないプロトコル処理（ファームの Comm の PC 側版）。
//   Feed: 受けたバイト列をフレームに分け、下りの指令を RobotModel に渡す
//   Tick: JOINT_STATE を毎回、INFO と SYSTEM を初回と 1 s ごとに書き出す
// pty も ROS も知らない。テストはバイト列で行う
#ifndef RABBITS_SIM__FAKE_FIRMWARE_HPP_
#define RABBITS_SIM__FAKE_FIRMWARE_HPP_

#include <cstddef>
#include <cstdint>

#include <rabcl/protocol/parser.hpp>
#include <rabcl/protocol/protocol.hpp>

#include "rabbits_sim/robot_model.hpp"

namespace rabbits_sim
{

class FakeFirmware
{
public:
  explicit FakeFirmware(RobotModel & model);

  // 下りのバイト列。復号できたフレームごとに model.OnCommandReceived(now_s)
  void Feed(const uint8_t * data, size_t len, double now_s);
  // 上りのフレームを out に連結して書き、総バイト数を返す（cap に収まる分だけ）
  size_t Tick(double now_s, uint8_t * out, size_t cap);

  uint32_t rx_frames() const {return rx_frames_;}
  uint32_t crc_errors() const {return parser_.crc_errors();}

private:
  static constexpr double INFO_PERIOD_S = 1.0;

  static void OnFrame(
    rabcl::protocol::Type type, const uint8_t * payload, uint8_t len, void * ctx);
  void HandleFrame(rabcl::protocol::Type type, const uint8_t * payload, uint8_t len);

  RobotModel & model_;
  rabcl::protocol::Parser parser_;
  double feed_now_s_ = 0.0;
  bool info_sent_ = false;
  double last_info_s_ = 0.0;
  uint32_t rx_frames_ = 0;
};

}  // namespace rabbits_sim

#endif  // RABBITS_SIM__FAKE_FIRMWARE_HPP_
