// gamma の偽ファームノード。PtyLink（仮想シリアル）、FakeFirmware（プロトコル）、GammaModel（機体）をつなぐだけ。
// ROS のトピックは持たない。定数は params ファイル（config/gamma.yaml）で渡す。未設定ならエラーで終了
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rabcl/protocol/protocol.hpp>

#include "rabbits_sim/fake_firmware.hpp"
#include "rabbits_sim/gamma_model.hpp"
#include "rabbits_sim/pty_link.hpp"

namespace rabbits_sim
{

class GammaFakeNode : public rclcpp::Node
{
public:
  explicit GammaFakeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gamma_fake_node", options)
  {
    const std::string link_path =
      this->declare_parameter<std::string>("link_path", "/tmp/gamma_fake");
    const int tick_hz = static_cast<int>(this->declare_parameter<int64_t>("tick_hz", 100));

    GammaConfig config = {};
    if (!LoadConfig(config)) {
      return;
    }
    model_ = std::make_unique<GammaModel>(config);
    firmware_ = std::make_unique<FakeFirmware>(*model_);

    if (!link_.Open(link_path)) {
      RCLCPP_ERROR(this->get_logger(), "could not open a pty / create link %s", link_path.c_str());
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "fake gamma on %s (link %s)", link_.slave_path().c_str(),
      link_.link_path().c_str());

    start_ = std::chrono::steady_clock::now();
    last_tick_s_ = 0.0;
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(1000000 / std::max(tick_hz, 1)),
      std::bind(&GammaFakeNode::Tick, this));
    ok_ = true;
  }

  bool ok() const {return ok_;}

private:
  static constexpr double LOG_PERIOD_S = 1.0;

  template<typename T>
  bool Require(const std::string & name, T & out)
  {
    try {
      out = this->declare_parameter<T>(name);
      return true;
    } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
      RCLCPP_ERROR(
        this->get_logger(), "parameter '%s' is required (pass config/gamma.yaml)", name.c_str());
    } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
      RCLCPP_ERROR(
        this->get_logger(), "parameter '%s' is required (pass config/gamma.yaml)", name.c_str());
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
      RCLCPP_ERROR(this->get_logger(), "parameter '%s': %s", name.c_str(), e.what());
    }
    return false;
  }

  // [x, z] の 2 要素パラメータ
  bool RequirePair(const std::string & name, float & x, float & z)
  {
    std::vector<double> v;
    if (!Require(name, v)) {
      return false;
    }
    if (v.size() != 2) {
      RCLCPP_ERROR(this->get_logger(), "parameter '%s' must have 2 values [x, z]", name.c_str());
      return false;
    }
    x = static_cast<float>(v[0]);
    z = static_cast<float>(v[1]);
    return true;
  }

  bool LoadConfig(GammaConfig & c)
  {
    double v = 0.0;
    std::vector<double> azimuth;
    bool ok = true;
    // 脚リンクの寸法（rabbits_description の config/gamma_linkage.yaml を launch で渡す）
    rabcl::LegLinkageConfig & l = c.posture.linkage;
    ok = Require("linkage.crank_length", v) && ok;
    l.crank_length = static_cast<float>(v);
    ok = Require("linkage.crank_angle_zero", v) && ok;
    l.crank_angle_zero = static_cast<float>(v);
    ok = Require("linkage.coupler_length", v) && ok;
    l.coupler_length = static_cast<float>(v);
    ok = Require("linkage.bar_length", v) && ok;
    l.bar_length = static_cast<float>(v);
    ok = RequirePair("linkage.bar_pivot", l.bar_pivot_x, l.bar_pivot_z) && ok;
    ok = RequirePair("linkage.coupler_pin", l.coupler_pin_x, l.coupler_pin_z) && ok;
    ok = RequirePair("linkage.wheel", l.wheel_x, l.wheel_z) && ok;
    ok = Require("linkage.theta_max", v) && ok;
    l.theta_max = static_cast<float>(v);
    ok = Require("linkage.drop_min", v) && ok;
    c.posture.drop_min = static_cast<float>(v);
    ok = Require("linkage.drop_max", v) && ok;
    c.posture.drop_max = static_cast<float>(v);
    ok = Require("linkage.tilt_radius", v) && ok;
    c.posture.tilt_radius = static_cast<float>(v);
    ok = Require("leg.azimuth", azimuth) && ok;
    if (azimuth.size() != 4) {
      RCLCPP_ERROR(
        this->get_logger(), "parameter 'leg.azimuth' must have 4 values (FR, FL, BR, BL)");
      ok = false;
    } else {
      for (size_t i = 0; i < 4; ++i) {
        c.posture.leg_azimuth[i] = static_cast<float>(azimuth[i]);
      }
    }
    ok = Require("leg.ramp_rate", v) && ok;
    c.leg_ramp_rate = static_cast<float>(v);
    ok = Require("wheel.wheel_d", c.wheel_d) && ok;
    ok = Require("wheel.body_d", c.body_d) && ok;
    ok = Require("wheel.sign", v) && ok;
    c.wheel_sign = static_cast<float>(v);
    ok = Require("chassis.vel_max", v) && ok;
    c.chassis_vel_max = static_cast<float>(v);
    ok = Require("chassis.rot_max", v) && ok;
    c.chassis_rot_max = static_cast<float>(v);
    ok = Require("chassis.spin_speed", v) && ok;
    c.spin_speed = static_cast<float>(v);
    ok = Require("watchdog_s", v) && ok;
    c.watchdog_s = static_cast<float>(v);
    c.follow_neck_pitch_roll = this->declare_parameter<bool>("follow_neck_pitch_roll", false);
    return ok;
  }

  double NowS() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
  }

  void Tick()
  {
    const double now = NowS();
    const double dt = now - last_tick_s_;
    last_tick_s_ = now;

    uint8_t rx[512];
    int n;
    while ((n = link_.Read(rx, sizeof(rx))) > 0) {
      firmware_->Feed(rx, static_cast<size_t>(n), now);
    }
    model_->Update(now, dt);
    uint8_t tx[rabcl::protocol::MAX_FRAME * 3];
    const size_t len = firmware_->Tick(now, tx, sizeof(tx));
    link_.Write(tx, len);

    if (now - last_log_s_ >= LOG_PERIOD_S) {
      last_log_s_ = now;
      RCLCPP_INFO(
        this->get_logger(),
        "rx_frames=%u crc_errors=%u %s | legs %.2f %.2f %.2f %.2f | yaw %.2f",
        firmware_->rx_frames(), firmware_->crc_errors(),
        model_->command_lost() ? "LOST" : "ok",
        static_cast<double>(model_->joint_angle(GammaModel::LEG_FR)),
        static_cast<double>(model_->joint_angle(GammaModel::LEG_FL)),
        static_cast<double>(model_->joint_angle(GammaModel::LEG_BR)),
        static_cast<double>(model_->joint_angle(GammaModel::LEG_BL)),
        static_cast<double>(model_->joint_angle(GammaModel::YAW)));
    }
  }

  PtyLink link_;
  std::unique_ptr<GammaModel> model_;
  std::unique_ptr<FakeFirmware> firmware_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point start_;
  double last_tick_s_ = 0.0;
  double last_log_s_ = 0.0;
  bool ok_ = false;
};

}  // namespace rabbits_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rabbits_sim::GammaFakeNode>();
  int status = 0;
  if (node->ok()) {
    rclcpp::spin(node);
  } else {
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
