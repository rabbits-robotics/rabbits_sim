// gamma の偽ファーム: ファーム（gamma_turret_fw の Controller）がやる変換を rabcl の同じ部品で再現し、
// 関節角を返す。モータは指令に即応答（脚のランプだけが時間変化を作る）。IMU は無い前提。
// 定数は GammaConfig（値は config/gamma.yaml。gamma_turret_fw の src/robot.hpp と対で更新する）
#ifndef RABBITS_SIM__GAMMA_MODEL_HPP_
#define RABBITS_SIM__GAMMA_MODEL_HPP_

#include <cstdint>

#include <rabcl/controller/body_attitude.hpp>
#include <rabcl/controller/body_posture.hpp>
#include <rabcl/controller/command_watchdog.hpp>
#include <rabcl/controller/gimbal.hpp>
#include <rabcl/controller/leg_damper.hpp>
#include <rabcl/controller/omni_drive.hpp>

#include "rabbits_sim/robot_model.hpp"

namespace rabbits_sim
{

struct GammaConfig
{
  float leg_angle_min;      // [rad] 脚の使用範囲（0 側が最低車高）
  float leg_angle_max;
  float leg_tilt_gain;      // [rad/rad]
  float leg_azimuth[4];     // [rad] FR, FL, BR, BL
  float leg_ramp_rate;      // [rad/s]
  double wheel_d;           // [m]
  double body_d;            // [m]
  float wheel_sign;         // 車輪の回転向き（ファームの WHEEL_SIGN）
  float chassis_vel_max;    // [m/s]
  float chassis_rot_max;    // [rad/s]
  float spin_speed;         // [rad/s] 無限回転
  float watchdog_s;         // [s] 途絶判定
  bool follow_neck_pitch_roll;  // true: JOINT_CMD の pitch / roll をそのまま関節角に（確認用）
};

class GammaModel : public RobotModel
{
public:
  // 関節番号（rabbits_bridge config/gamma.yaml の並び）
  enum Joint : uint8_t
  {
    YAW = 0,
    PITCH,
    ROLL,
    WHEEL_FR,
    WHEEL_FL,
    WHEEL_BR,
    WHEEL_BL,
    LEG_FR,
    LEG_FL,
    LEG_BR,
    LEG_BL,
    JOINT_COUNT
  };
  enum Mode : uint8_t
  {
    MODE_LOAD = 0,
    MODE_FIRE,
    MODE_SPEED,
    MODE_CHASSIS,
    MODE_STABILIZE,
    MODE_COUNT
  };

  explicit GammaModel(const GammaConfig & config);

  uint8_t joint_count() const override {return JOINT_COUNT;}
  const char * firmware_id() const override {return "gamma-sm";}
  void SetChassisVel(const rabcl::protocol::ChassisVel & vel) override;
  void SetBodyPose(const rabcl::protocol::BodyPose & pose) override;
  void SetJointCmd(const rabcl::protocol::JointCmd & cmd) override;
  void SetMode(const rabcl::protocol::ModeCmd & mode) override;
  void OnCommandReceived(double now_s) override;
  void Update(double now_s, double dt) override;
  void GetJointStates(rabcl::protocol::JointState * out) const override;
  uint32_t error_flags() const override;

  float joint_angle(uint8_t id) const;
  bool command_lost() const {return lost_;}

private:
  static constexpr float MAX_DT_S = 0.05f;  // ファームと同じ上限
  static constexpr uint32_t ERROR_IMU_INVALID = 1u << 4;
  static constexpr uint32_t ERROR_CMD_LOST = 1u << 5;

  float ChassisWz() const;

  GammaConfig config_;
  rabcl::OmniDrive omni_;
  rabcl::BodyPosture posture_;
  rabcl::LegDamper legs_[4];
  rabcl::BodyAttitude attitude_;
  rabcl::Gimbal gimbal_;
  rabcl::CommandWatchdog watchdog_;

  rabcl::protocol::ChassisVel chassis_vel_ = {};
  rabcl::protocol::BodyPose body_pose_ = {};
  bool body_pose_present_ = false;
  float neck_cmd_[3] = {0.0f, 0.0f, 0.0f};  // yaw, pitch, roll の JOINT_CMD
  bool yaw_cmd_present_ = false;
  bool infinite_rotation_ = false;
  bool lost_ = true;

  float angle_[JOINT_COUNT] = {};
  float velocity_[JOINT_COUNT] = {};
};

}  // namespace rabbits_sim

#endif  // RABBITS_SIM__GAMMA_MODEL_HPP_
