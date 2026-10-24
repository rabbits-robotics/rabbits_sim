#include <gtest/gtest.h>

#include <cstring>
#include <utility>
#include <vector>

#include <rabcl/protocol/parser.hpp>
#include <rabcl/protocol/protocol.hpp>

#include "rabbits_sim/fake_firmware.hpp"

namespace rabbits_sim
{

namespace proto = rabcl::protocol;

// 受けた指令を記録するだけの機体モデル（関節 3 つ、角度 0.1 / 0.2 / 0.3、error_flags 0x20）
class RecordingModel : public RobotModel
{
public:
  uint8_t joint_count() const override {return 3;}
  const char * firmware_id() const override {return "rec";}
  void SetChassisVel(const proto::ChassisVel & vel) override
  {
    chassis_vel = vel;
    ++chassis_vel_count;
  }
  void SetBodyPose(const proto::BodyPose & pose) override
  {
    body_pose = pose;
    ++body_pose_count;
  }
  void SetJointCmd(const proto::JointCmd & cmd) override {joint_cmds.push_back(cmd);}
  void SetMode(const proto::ModeCmd & mode) override {modes.push_back(mode);}
  void OnCommandReceived(double now_s) override
  {
    last_command_s = now_s;
    ++command_count;
  }
  void Update(double, double) override {}
  void GetJointStates(proto::JointState * out) const override
  {
    for (uint8_t i = 0; i < 3; ++i) {
      out[i] = {};
      out[i].id = i;
      out[i].position = 0.1f * static_cast<float>(i + 1);
    }
  }
  uint32_t error_flags() const override {return 0x20;}

  proto::ChassisVel chassis_vel = {};
  int chassis_vel_count = 0;
  proto::BodyPose body_pose = {};
  int body_pose_count = 0;
  std::vector<proto::JointCmd> joint_cmds;
  std::vector<proto::ModeCmd> modes;
  double last_command_s = -1.0;
  int command_count = 0;
};

using Frames = std::vector<std::pair<proto::Type, std::vector<uint8_t>>>;

Frames ParseFrames(const uint8_t * bytes, size_t len)
{
  Frames frames;
  proto::Parser parser;
  parser.Feed(
    bytes, len,
    [](proto::Type type, const uint8_t * payload, uint8_t plen, void * ctx) {
      static_cast<Frames *>(ctx)->emplace_back(
        type, std::vector<uint8_t>(payload, payload + plen));
    }, &frames);
  return frames;
}

int Count(const Frames & frames, proto::Type type)
{
  int n = 0;
  for (const auto & f : frames) {
    if (f.first == type) {
      ++n;
    }
  }
  return n;
}

class FakeFirmwareTest : public ::testing::Test
{
protected:
  Frames Tick(double now_s)
  {
    uint8_t buf[proto::MAX_FRAME * 4];
    const size_t n = fw_.Tick(now_s, buf, sizeof(buf));
    return ParseFrames(buf, n);
  }

  RecordingModel model_;
  FakeFirmware fw_{model_};
};

TEST_F(FakeFirmwareTest, TickEmitsAllJoints)
{
  const Frames frames = Tick(0.0);
  ASSERT_EQ(Count(frames, proto::Type::JOINT_STATE), 1);
  for (const auto & f : frames) {
    if (f.first != proto::Type::JOINT_STATE) {
      continue;
    }
    proto::JointState joints[proto::MAX_JOINT_STATES];
    uint8_t count = 0;
    ASSERT_TRUE(
      proto::DecodeJointState(
        f.second.data(), static_cast<uint8_t>(f.second.size()), joints, proto::MAX_JOINT_STATES,
        count));
    ASSERT_EQ(count, 3);
    EXPECT_EQ(joints[0].id, 0);
    EXPECT_EQ(joints[2].id, 2);
    EXPECT_FLOAT_EQ(joints[0].position, 0.1f);
    EXPECT_FLOAT_EQ(joints[1].position, 0.2f);
    EXPECT_FLOAT_EQ(joints[2].position, 0.3f);
  }
}

TEST_F(FakeFirmwareTest, FirstTickAlsoEmitsInfoAndSystem)
{
  const Frames frames = Tick(0.0);
  ASSERT_EQ(Count(frames, proto::Type::INFO), 1);
  ASSERT_EQ(Count(frames, proto::Type::SYSTEM), 1);
  for (const auto & f : frames) {
    if (f.first == proto::Type::INFO) {
      proto::Info info = {};
      ASSERT_TRUE(
        proto::DecodeInfo(f.second.data(), static_cast<uint8_t>(f.second.size()), info));
      EXPECT_EQ(info.protocol_version, proto::VERSION);
      EXPECT_EQ(info.joint_count, 3);
      EXPECT_STREQ(info.firmware_id, "rec");
    } else if (f.first == proto::Type::SYSTEM) {
      proto::System system = {};
      ASSERT_TRUE(
        proto::DecodeSystem(f.second.data(), static_cast<uint8_t>(f.second.size()), system));
      EXPECT_EQ(system.error_flags, 0x20u);
      EXPECT_EQ(system.uptime_ms, 0u);
    }
  }
}

TEST_F(FakeFirmwareTest, InfoSystemOncePerSecond)
{
  int info = 0;
  int system = 0;
  int joint_state = 0;
  for (int i = 0; i <= 150; ++i) {  // 0.00 〜 1.50 s
    const Frames frames = Tick(0.01 * i);
    info += Count(frames, proto::Type::INFO);
    system += Count(frames, proto::Type::SYSTEM);
    joint_state += Count(frames, proto::Type::JOINT_STATE);
  }
  EXPECT_EQ(info, 2);  // 0.0 と 1.0
  EXPECT_EQ(system, 2);
  EXPECT_EQ(joint_state, 151);
}

TEST_F(FakeFirmwareTest, ChassisVelReachesModel)
{
  uint8_t buf[proto::MAX_FRAME];
  const size_t n = proto::EncodeChassisVel({0.5f, -0.25f, 1.5f}, buf, sizeof(buf));
  fw_.Feed(buf, n, 2.0);
  EXPECT_EQ(model_.chassis_vel_count, 1);
  EXPECT_FLOAT_EQ(model_.chassis_vel.vx, 0.5f);
  EXPECT_FLOAT_EQ(model_.chassis_vel.vy, -0.25f);
  EXPECT_FLOAT_EQ(model_.chassis_vel.wz, 1.5f);
  EXPECT_EQ(model_.command_count, 1);
  EXPECT_DOUBLE_EQ(model_.last_command_s, 2.0);
  EXPECT_EQ(fw_.rx_frames(), 1u);
}

TEST_F(FakeFirmwareTest, BodyPoseJointCmdModeReachModel)
{
  uint8_t buf[proto::MAX_FRAME];
  size_t n = proto::EncodeBodyPose({0.5f, 0.1f, -0.2f}, buf, sizeof(buf));
  fw_.Feed(buf, n, 1.0);
  const proto::JointCmd cmds[2] = {{0, 0.5f, 0, 0, 0, 0}, {7, 1.0f, 0, 0, 0, 0}};
  n = proto::EncodeJointCmd(cmds, 2, buf, sizeof(buf));
  fw_.Feed(buf, n, 1.1);
  const proto::ModeCmd modes[2] = {{3, 1}, {4, 1}};
  n = proto::EncodeMode(modes, 2, buf, sizeof(buf));
  fw_.Feed(buf, n, 1.2);

  EXPECT_EQ(model_.body_pose_count, 1);
  EXPECT_FLOAT_EQ(model_.body_pose.height, 0.5f);
  EXPECT_FLOAT_EQ(model_.body_pose.pitch, -0.2f);
  ASSERT_EQ(model_.joint_cmds.size(), 2u);
  EXPECT_EQ(model_.joint_cmds[1].id, 7);
  EXPECT_FLOAT_EQ(model_.joint_cmds[1].position, 1.0f);
  ASSERT_EQ(model_.modes.size(), 2u);
  EXPECT_EQ(model_.modes[0].id, 3);
  EXPECT_EQ(model_.modes[1].value, 1);
  EXPECT_EQ(model_.command_count, 3);
  EXPECT_DOUBLE_EQ(model_.last_command_s, 1.2);
}

TEST_F(FakeFirmwareTest, CorruptedCrcIsDropped)
{
  uint8_t buf[proto::MAX_FRAME];
  const size_t n = proto::EncodeChassisVel({1.0f, 0.0f, 0.0f}, buf, sizeof(buf));
  buf[n - 1] ^= 0xFF;
  fw_.Feed(buf, n, 0.5);
  EXPECT_EQ(model_.chassis_vel_count, 0);
  EXPECT_EQ(model_.command_count, 0);
  EXPECT_EQ(fw_.crc_errors(), 1u);
}

TEST_F(FakeFirmwareTest, SplitFramesAreReassembled)
{
  uint8_t buf[proto::MAX_FRAME];
  const size_t n = proto::EncodeChassisVel({1.0f, 0.0f, 0.0f}, buf, sizeof(buf));
  fw_.Feed(buf, 4, 0.1);
  EXPECT_EQ(model_.chassis_vel_count, 0);
  fw_.Feed(buf + 4, n - 4, 0.2);
  EXPECT_EQ(model_.chassis_vel_count, 1);
  EXPECT_DOUBLE_EQ(model_.last_command_s, 0.2);
}

}  // namespace rabbits_sim

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
