// 偽ファームの「機体の振る舞い」の口。機体ごと（gamma、将来 beta）に実装する。
// FakeFirmware が下りの指令を渡し、関節状態と error_flags を読む。プロトコルも pty も知らない
#ifndef RABBITS_SIM__ROBOT_MODEL_HPP_
#define RABBITS_SIM__ROBOT_MODEL_HPP_

#include <cstdint>

#include <rabcl/protocol/protocol.hpp>

namespace rabbits_sim
{

class RobotModel
{
public:
  virtual ~RobotModel() = default;

  virtual uint8_t joint_count() const = 0;
  virtual const char * firmware_id() const = 0;  // 8 文字以内（INFO の firmware_id）

  // 下りの指令（FakeFirmware が復号して渡す）
  virtual void SetChassisVel(const rabcl::protocol::ChassisVel & vel) = 0;
  virtual void SetBodyPose(const rabcl::protocol::BodyPose & pose) = 0;
  virtual void SetJointCmd(const rabcl::protocol::JointCmd & cmd) = 0;  // 配列の要素ごとに 1 回
  virtual void SetMode(const rabcl::protocol::ModeCmd & mode) = 0;      // 同上
  // 復号できた下りフレームごとに 1 回（途絶監視用）
  virtual void OnCommandReceived(double now_s) = 0;

  // 関節の状態を dt [s] だけ進める
  virtual void Update(double now_s, double dt) = 0;
  // joint_count 個を out に書く。id は 0〜joint_count-1
  virtual void GetJointStates(rabcl::protocol::JointState * out) const = 0;
  virtual uint32_t error_flags() const = 0;
};

}  // namespace rabbits_sim

#endif  // RABBITS_SIM__ROBOT_MODEL_HPP_
