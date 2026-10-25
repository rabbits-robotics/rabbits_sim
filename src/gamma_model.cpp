#include "rabbits_sim/gamma_model.hpp"

#include <algorithm>

#include <rabcl/utils/utils.hpp>

namespace rabbits_sim
{

namespace
{

rabcl::LegDamperConfig LegConfig(const GammaConfig & c)
{
  // kp / kd / tff はモータの話なので偽ファームでは使わない。ランプだけ
  return {0.0f, 0.0f, 0.0f, 0.0f, c.leg_ramp_rate};
}

}  // namespace

GammaModel::GammaModel(const GammaConfig & config)
: config_(config),
  omni_(config.wheel_d, config.body_d),
  posture_(config.posture),
  legs_{rabcl::LegDamper(LegConfig(config)), rabcl::LegDamper(LegConfig(config)),
    rabcl::LegDamper(LegConfig(config)), rabcl::LegDamper(LegConfig(config))},
  watchdog_(config.watchdog_s)
{
  for (size_t i = 0; i < 4; ++i) {
    legs_[i].Reset(0.0f);  // モータ原点（メカスト = 最低車高）から始まる。実機の電源投入と同じ
    angle_[LEG_FR + i] = 0.0f;
  }
}

void GammaModel::SetChassisVel(const rabcl::protocol::ChassisVel & vel)
{
  chassis_vel_ = vel;
}

void GammaModel::SetBodyPose(const rabcl::protocol::BodyPose & pose)
{
  body_pose_ = pose;
  body_pose_present_ = true;
}

void GammaModel::SetJointCmd(const rabcl::protocol::JointCmd & cmd)
{
  if (cmd.id > ROLL) {
    return;  // 首以外の JOINT_CMD はファームも使わない
  }
  neck_cmd_[cmd.id] = cmd.position;
  if (cmd.id == YAW) {
    yaw_cmd_present_ = true;
  }
}

void GammaModel::SetMode(const rabcl::protocol::ModeCmd & mode)
{
  if (mode.id == MODE_CHASSIS) {
    infinite_rotation_ = mode.value == 1;
  }
  // 他のモードは保持するだけ（射出・装填・歩き走り・安定化は関節に現れない）
}

void GammaModel::OnCommandReceived(double now_s)
{
  watchdog_.Feed(static_cast<float>(now_s));
}

float GammaModel::ChassisWz() const
{
  if (lost_) {
    return 0.0f;
  }
  float wz = std::clamp(chassis_vel_.wz, -config_.chassis_rot_max, config_.chassis_rot_max);
  if (infinite_rotation_) {
    wz += config_.spin_speed;
  }
  return wz;
}

void GammaModel::Update(double now_s, double dt_in)
{
  const float dt = std::clamp(static_cast<float>(dt_in), 0.0f, MAX_DT_S);
  lost_ = watchdog_.Expired(static_cast<float>(now_s));

  // 車輪: 視点基準の速度分配 + 無限回転。途絶で 0
  float vx = 0.0f;
  float vy = 0.0f;
  if (!lost_) {
    vx = std::clamp(chassis_vel_.vx, -config_.chassis_vel_max, config_.chassis_vel_max);
    vy = std::clamp(chassis_vel_.vy, -config_.chassis_vel_max, config_.chassis_vel_max);
  }
  const float wz = ChassisWz();
  const float yaw_angle = angle_[YAW];
  double wheel[4];
  omni_.CalcVel(vx, vy, wz, wheel[0], wheel[1], wheel[2], wheel[3], yaw_angle);
  for (size_t i = 0; i < 4; ++i) {
    const float omega = config_.wheel_sign * static_cast<float>(wheel[i]);
    velocity_[WHEEL_FR + i] = omega;
    angle_[WHEEL_FR + i] = rabcl::Utils::WrapPi(angle_[WHEEL_FR + i] + omega * dt);
  }

  // 脚: 最後の BODY_POSE の姿勢へランプ（途絶でも進み切る = ファームと同じ）
  float target[4];
  if (body_pose_present_) {
    posture_.Solve(body_pose_.height, body_pose_.roll, body_pose_.pitch, yaw_angle, target);
  } else {
    for (size_t i = 0; i < 4; ++i) {
      target[i] = legs_[i].ramped_target();
    }
  }
  for (size_t i = 0; i < 4; ++i) {
    const float before = angle_[LEG_FR + i];
    angle_[LEG_FR + i] = legs_[i].Update(target[i], before, 0.0f, dt, 0.0f).pos;  // 荷重分は無し
    velocity_[LEG_FR + i] = dt > 0.0f ? (angle_[LEG_FR + i] - before) / dt : 0.0f;
  }

  // 首 Yaw: 無限回転中だけ胴体の向き（IMU 無しなので旋回速度の積分）を追って打ち消す。
  // 普通の旋回（L2 / R2）では据え置き = 首は胴体と一緒に回る（ファームと同じ）
  attitude_.SetYawTracking(infinite_rotation_);
  attitude_.UpdateFromChassisRate(wz, dt);
  const rabcl::Attitude body = attitude_.Get();
  if (yaw_cmd_present_) {
    angle_[YAW] = gimbal_.Solve({neck_cmd_[YAW], 0.0f, 0.0f}, body).yaw;
  }
  // 首 Pitch / Roll: ファームは機構決定まで送らない → 0。確認用に追従させる設定あり
  angle_[PITCH] = config_.follow_neck_pitch_roll ? neck_cmd_[PITCH] : 0.0f;
  angle_[ROLL] = config_.follow_neck_pitch_roll ? neck_cmd_[ROLL] : 0.0f;
}

void GammaModel::GetJointStates(rabcl::protocol::JointState * out) const
{
  for (uint8_t id = 0; id < JOINT_COUNT; ++id) {
    out[id] = {};
    out[id].id = id;
    out[id].position = angle_[id];
    out[id].velocity = velocity_[id];
  }
}

uint32_t GammaModel::error_flags() const
{
  return ERROR_IMU_INVALID | (lost_ ? ERROR_CMD_LOST : 0u);
}

float GammaModel::joint_angle(uint8_t id) const
{
  return id < JOINT_COUNT ? angle_[id] : 0.0f;
}

}  // namespace rabbits_sim
