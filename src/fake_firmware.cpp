#include "rabbits_sim/fake_firmware.hpp"

#include <cstring>

namespace rabbits_sim
{

namespace proto = rabcl::protocol;

FakeFirmware::FakeFirmware(RobotModel & model)
: model_(model)
{
}

void FakeFirmware::Feed(const uint8_t * data, size_t len, double now_s)
{
  feed_now_s_ = now_s;
  parser_.Feed(data, len, OnFrame, this);
}

void FakeFirmware::OnFrame(proto::Type type, const uint8_t * payload, uint8_t len, void * ctx)
{
  static_cast<FakeFirmware *>(ctx)->HandleFrame(type, payload, len);
}

void FakeFirmware::HandleFrame(proto::Type type, const uint8_t * payload, uint8_t len)
{
  bool decoded = false;
  switch (type) {
    case proto::Type::CHASSIS_VEL: {
        proto::ChassisVel vel;
        if (proto::DecodeChassisVel(payload, len, vel)) {
          model_.SetChassisVel(vel);
          decoded = true;
        }
        break;
      }
    case proto::Type::BODY_POSE: {
        proto::BodyPose pose;
        if (proto::DecodeBodyPose(payload, len, pose)) {
          model_.SetBodyPose(pose);
          decoded = true;
        }
        break;
      }
    case proto::Type::JOINT_CMD: {
        proto::JointCmd cmds[proto::MAX_JOINT_CMDS];
        uint8_t count = 0;
        if (proto::DecodeJointCmd(payload, len, cmds, proto::MAX_JOINT_CMDS, count)) {
          for (uint8_t i = 0; i < count; ++i) {
            model_.SetJointCmd(cmds[i]);
          }
          decoded = true;
        }
        break;
      }
    case proto::Type::MODE: {
        proto::ModeCmd modes[proto::MAX_MODES];
        uint8_t count = 0;
        if (proto::DecodeMode(payload, len, modes, proto::MAX_MODES, count)) {
          for (uint8_t i = 0; i < count; ++i) {
            model_.SetMode(modes[i]);
          }
          decoded = true;
        }
        break;
      }
    default:
      break;  // 上りの種別は無視
  }
  if (decoded) {
    ++rx_frames_;
    model_.OnCommandReceived(feed_now_s_);
  }
}

size_t FakeFirmware::Tick(double now_s, uint8_t * out, size_t cap)
{
  size_t total = 0;

  proto::JointState joints[proto::MAX_JOINT_STATES] = {};
  const uint8_t count = model_.joint_count();
  if (count <= proto::MAX_JOINT_STATES) {
    model_.GetJointStates(joints);
    total += proto::EncodeJointState(joints, count, out + total, cap - total);
  }

  if (!info_sent_ || now_s - last_info_s_ >= INFO_PERIOD_S) {
    info_sent_ = true;
    last_info_s_ = now_s;

    proto::Info info = {};
    info.protocol_version = proto::VERSION;
    info.joint_count = count;
    // 末尾 0 埋め（protocol の約束）。8 文字以上は 7 文字に切る
    std::strncpy(info.firmware_id, model_.firmware_id(), sizeof(info.firmware_id) - 1);
    info.firmware_id[sizeof(info.firmware_id) - 1] = '\0';
    total += proto::EncodeInfo(info, out + total, cap - total);

    proto::System system = {};
    system.bus_voltage = 0.0f;
    system.error_flags = model_.error_flags();
    system.uptime_ms = static_cast<uint32_t>(now_s * 1000.0);
    total += proto::EncodeSystem(system, out + total, cap - total);
  }
  return total;
}

}  // namespace rabbits_sim
