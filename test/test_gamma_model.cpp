#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include <rabcl/controller/omni_drive.hpp>
#include <rabcl/protocol/protocol.hpp>
#include <rabcl/utils/utils.hpp>

#include "rabbits_sim/gamma_model.hpp"

namespace rabbits_sim
{

namespace proto = rabcl::protocol;

// 関節番号（rabbits_bridge config/gamma.yaml の並び）
constexpr uint8_t YAW = 0;
constexpr uint8_t PITCH = 1;
constexpr uint8_t WHEEL_FR = 3;
constexpr uint8_t WHEEL_BL = 6;
constexpr uint8_t LEG_FR = 7;
constexpr uint8_t LEG_FL = 8;
constexpr uint8_t LEG_BR = 9;
constexpr uint8_t LEG_BL = 10;
constexpr uint8_t MODE_CHASSIS = 3;
constexpr float PI = static_cast<float>(M_PI);
constexpr uint32_t FLAG_IMU_INVALID = 1u << 4;
constexpr uint32_t FLAG_CMD_LOST = 1u << 5;

GammaConfig TestConfig()
{
  GammaConfig c = {};
  c.leg_angle_min = 0.15f;
  c.leg_angle_max = 1.45f;
  c.leg_tilt_gain = std::sqrt(2.0f);  // rabcl の test_body_posture と同じ値（0.707 × √2 = 1）
  c.leg_azimuth[0] = -PI / 4.0f;
  c.leg_azimuth[1] = PI / 4.0f;
  c.leg_azimuth[2] = -3.0f * PI / 4.0f;
  c.leg_azimuth[3] = 3.0f * PI / 4.0f;
  c.leg_ramp_rate = 0.5f;
  c.wheel_d = 0.06;
  c.body_d = 0.28;
  c.wheel_sign = -1.0f;
  c.chassis_vel_max = 1.0f;
  c.chassis_rot_max = 3.0f;
  c.spin_speed = 5.0f;
  c.watchdog_s = 0.5f;
  c.follow_neck_pitch_roll = false;
  return c;
}

class GammaModelTest : public ::testing::Test
{
protected:
  static constexpr double DT = 0.01;

  void SendChassisVel(float vx, float vy, float wz)
  {
    model_.SetChassisVel({vx, vy, wz});
    model_.OnCommandReceived(now_);
  }
  void SendBodyPose(float h, float roll, float pitch)
  {
    model_.SetBodyPose({h, roll, pitch});
    model_.OnCommandReceived(now_);
  }
  void SendJointCmd(uint8_t id, float pos)
  {
    proto::JointCmd cmd = {};
    cmd.id = id;
    cmd.position = pos;
    model_.SetJointCmd(cmd);
    model_.OnCommandReceived(now_);
  }
  void SendMode(uint8_t id, uint8_t value)
  {
    model_.SetMode({id, value});
    model_.OnCommandReceived(now_);
  }
  // 指令を送り続けながら seconds 進める
  void StepAlive(double seconds)
  {
    const int n = static_cast<int>(std::lround(seconds / DT));
    for (int i = 0; i < n; ++i) {
      now_ += DT;
      model_.OnCommandReceived(now_);
      model_.Update(now_, DT);
    }
  }
  // 指令を止めて seconds 進める（途絶）
  void Step(double seconds)
  {
    const int n = static_cast<int>(std::lround(seconds / DT));
    for (int i = 0; i < n; ++i) {
      now_ += DT;
      model_.Update(now_, DT);
    }
  }
  float Angle(uint8_t id) const {return model_.joint_angle(id);}
  float Velocity(uint8_t id) const
  {
    proto::JointState js[11];
    model_.GetJointStates(js);
    return js[id].velocity;
  }

  GammaModel model_{TestConfig()};
  double now_ = 0.0;
};

TEST_F(GammaModelTest, StartsAtLowestHeightAndZeroNeck)
{
  EXPECT_EQ(model_.joint_count(), 11);
  EXPECT_STREQ(model_.firmware_id(), "gammasm");
  EXPECT_LE(std::strlen(model_.firmware_id()), 7u);  // INFO の firmware_id[8] に NUL 終端で収まる
  for (uint8_t id = LEG_FR; id <= LEG_BL; ++id) {
    EXPECT_FLOAT_EQ(Angle(id), 0.15f);
  }
  for (uint8_t id = YAW; id <= WHEEL_BL; ++id) {
    EXPECT_FLOAT_EQ(Angle(id), 0.0f);
  }
  proto::JointState js[11];
  model_.GetJointStates(js);
  EXPECT_EQ(js[10].id, 10);
  EXPECT_FLOAT_EQ(js[10].position, 0.15f);
}

TEST_F(GammaModelTest, BodyHeightRampsToBodyPostureTarget)
{
  SendBodyPose(0.5f, 0.0f, 0.0f);
  StepAlive(0.5);
  for (uint8_t id = LEG_FR; id <= LEG_BL; ++id) {
    EXPECT_GT(Angle(id), 0.15f);
    EXPECT_LT(Angle(id), 0.8f);
  }
  StepAlive(1.5);
  for (uint8_t id = LEG_FR; id <= LEG_BL; ++id) {
    EXPECT_NEAR(Angle(id), 0.80f, 1e-3f);
  }
}

TEST_F(GammaModelTest, RollPositiveLowersRightLegs)
{
  SendBodyPose(0.5f, 0.1f, 0.0f);
  StepAlive(2.0);
  EXPECT_NEAR(Angle(LEG_FR), 0.70f, 1e-3f);
  EXPECT_NEAR(Angle(LEG_FL), 0.90f, 1e-3f);
  EXPECT_NEAR(Angle(LEG_BR), 0.70f, 1e-3f);
  EXPECT_NEAR(Angle(LEG_BL), 0.90f, 1e-3f);
}

TEST_F(GammaModelTest, PitchIsCameraFrameWhenYawIs90deg)
{
  SendJointCmd(YAW, PI / 2.0f);
  StepAlive(0.1);
  ASSERT_NEAR(Angle(YAW), PI / 2.0f, 1e-5f);
  SendBodyPose(0.5f, 0.0f, 0.2f);  // カメラの前下がり = 車体の左下がり
  StepAlive(3.0);
  EXPECT_NEAR(Angle(LEG_FR), 1.00f, 1e-3f);
  EXPECT_NEAR(Angle(LEG_FL), 0.60f, 1e-3f);
  EXPECT_NEAR(Angle(LEG_BR), 1.00f, 1e-3f);
  EXPECT_NEAR(Angle(LEG_BL), 0.60f, 1e-3f);
}

TEST_F(GammaModelTest, ForwardVelocitySpinsWheels)
{
  SendChassisVel(0.5f, 0.0f, 0.0f);
  StepAlive(1.0);
  rabcl::OmniDrive omni(0.06, 0.28);
  double w[4];
  omni.CalcVel(0.5, 0.0, 0.0, w[0], w[1], w[2], w[3], 0.0);
  for (uint8_t i = 0; i < 4; ++i) {
    const float omega = -1.0f * static_cast<float>(w[i]);
    EXPECT_NEAR(Angle(WHEEL_FR + i), rabcl::Utils::WrapPi(omega * 1.0f), 1e-3f) << "wheel " << i;
    EXPECT_NEAR(Velocity(WHEEL_FR + i), omega, 1e-4f) << "wheel " << i;
    EXPECT_NE(omega, 0.0f);
  }
}

TEST_F(GammaModelTest, WheelAngleWrapsWithinPi)
{
  SendChassisVel(1.0f, 0.0f, 0.0f);
  StepAlive(10.0);
  for (uint8_t id = WHEEL_FR; id <= WHEEL_BL; ++id) {
    EXPECT_LE(std::fabs(Angle(id)), PI);
  }
}

TEST_F(GammaModelTest, InfiniteRotationSpinsWhenStopped)
{
  SendChassisVel(0.0f, 0.0f, 0.0f);
  SendMode(MODE_CHASSIS, 1);
  StepAlive(0.5);
  for (uint8_t id = WHEEL_FR; id <= WHEEL_BL; ++id) {
    EXPECT_GT(std::fabs(Velocity(id)), 1.0f);
  }
}

TEST_F(GammaModelTest, LossStopsWheelsAndFlags)
{
  SendChassisVel(1.0f, 0.0f, 0.0f);
  StepAlive(0.2);
  EXPECT_FALSE(model_.command_lost());
  EXPECT_EQ(model_.error_flags(), FLAG_IMU_INVALID);
  Step(0.6);
  EXPECT_TRUE(model_.command_lost());
  EXPECT_EQ(model_.error_flags(), FLAG_IMU_INVALID | FLAG_CMD_LOST);
  for (uint8_t id = WHEEL_FR; id <= WHEEL_BL; ++id) {
    EXPECT_FLOAT_EQ(Velocity(id), 0.0f);
  }
}

TEST_F(GammaModelTest, LossKeepsRampingToLastPoseAndResumes)
{
  SendBodyPose(1.0f, 0.0f, 0.0f);
  StepAlive(0.2);
  Step(3.0);  // 途絶中も最後の BODY_POSE へ進み切る
  EXPECT_TRUE(model_.command_lost());
  for (uint8_t id = LEG_FR; id <= LEG_BL; ++id) {
    EXPECT_NEAR(Angle(id), 1.45f, 1e-3f);
  }
  SendBodyPose(0.0f, 0.0f, 0.0f);
  StepAlive(0.1);
  EXPECT_FALSE(model_.command_lost());
  for (uint8_t id = LEG_FR; id <= LEG_BL; ++id) {
    EXPECT_LT(Angle(id), 1.45f);
  }
}

TEST_F(GammaModelTest, YawFollowsJointCmd)
{
  SendJointCmd(YAW, 0.5f);
  StepAlive(0.05);
  EXPECT_NEAR(Angle(YAW), 0.5f, 1e-5f);
}

TEST_F(GammaModelTest, YawCountersBodyRotation)
{
  SendJointCmd(YAW, 0.0f);
  SendMode(MODE_CHASSIS, 1);
  StepAlive(0.1);  // 胴体は spin_speed で 0.5 rad 回る → 首は −0.5
  EXPECT_NEAR(Angle(YAW), -0.5f, 1e-3f);
}

TEST_F(GammaModelTest, YawTurnsWithBodyOnNormalRotation)
{
  SendJointCmd(YAW, 0.3f);
  SendChassisVel(0.0f, 0.0f, 1.0f);  // L2 / R2 の旋回: 胴体は回るが首は胴体基準の目標のまま
  StepAlive(0.5);
  EXPECT_NEAR(Angle(YAW), 0.3f, 1e-5f);
}

TEST_F(GammaModelTest, YawCountersOnlyInfiniteRotationPart)
{
  SendJointCmd(YAW, 0.0f);
  SendChassisVel(0.0f, 0.0f, 1.0f);
  StepAlive(0.5);  // 普通の旋回: 首は 0 のまま
  EXPECT_NEAR(Angle(YAW), 0.0f, 1e-5f);
  SendMode(MODE_CHASSIS, 1);
  StepAlive(0.1);  // 無限回転中は (spin_speed + 1.0) × 0.1 だけ逆回り
  EXPECT_NEAR(Angle(YAW), -0.6f, 1e-3f);
  SendMode(MODE_CHASSIS, 0);
  StepAlive(0.5);  // 抜けたら据え置き
  EXPECT_NEAR(Angle(YAW), -0.6f, 1e-3f);
}

TEST_F(GammaModelTest, NeckPitchRollHoldZeroByDefault)
{
  SendJointCmd(PITCH, 0.3f);
  StepAlive(0.05);
  EXPECT_FLOAT_EQ(Angle(PITCH), 0.0f);

  GammaConfig c = TestConfig();
  c.follow_neck_pitch_roll = true;
  GammaModel following(c);
  proto::JointCmd cmd = {};
  cmd.id = PITCH;
  cmd.position = 0.3f;
  following.SetJointCmd(cmd);
  following.OnCommandReceived(0.0);
  following.Update(0.01, 0.01);
  EXPECT_FLOAT_EQ(following.joint_angle(PITCH), 0.3f);
}

TEST_F(GammaModelTest, JointCmdForNonNeckIsIgnored)
{
  SendJointCmd(LEG_FR, 1.0f);
  StepAlive(0.5);
  EXPECT_FLOAT_EQ(Angle(LEG_FR), 0.15f);
}

}  // namespace rabbits_sim

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
