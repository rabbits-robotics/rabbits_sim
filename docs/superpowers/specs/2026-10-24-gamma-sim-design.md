# gamma の偽ファームと rviz 可視化 設計

2026-10-24。対象リポジトリ: rabbits_sim（新規）、rabbits_description（gamma の URDF 追加）、rabbits_bringup（新規、横断 launch）。

## 1. 目的

実機なしで、PC 側（rabbits_interface → rabbits_bridge）が作る指令 `reference_command` が正しいことを、rviz のロボットモデルの **関節の動き** で確認する。ファームがやるはずの変換（車高・ロール・ピッチ → 脚の角度、視点基準の速度分配、無限回転、途絶時の停止、首の視点追従）も PC 上で再現し、rabcl の制御部品のロジック確認も兼ねる。

背景: 2026-10-23〜24 に gamma の制御本体（設計書 gamma_turret_fw `docs/superpowers/specs/2026-10-23-gamma-control-design.md`）を実装した。rabcl と ROS 2 側はコミット済み、ファームはビルドのみで実機確認待ち。実機の前に PC 側だけで動作を目で確かめたい。以前（beta）は interface が指令値から擬似 `joint_states` を出して rviz で見ていたが、今は `joint_states` はファームのフィードバックから bridge が出す構成なので、ファームの代わりが必要。

## 2. ゴール・非ゴール・アンチゴール

ゴール:

- ゲームパッドまたはゲーム入力を操作すると、rviz 上の gamma モデルの **全関節**（首 Yaw / Pitch / Roll、脚 4、車輪 4）が、実機のファームが動かすはずの通りに動く。
- rabbits_bridge は実機と同じバイナリ・同じ設定（`config/gamma.yaml`）で動き、違いは launch で渡す `device` が実機のポートか仮想シリアルかだけ。
- 偽ファームの中身は rabcl の部品（`BodyPosture` / `LegDamper` / `OmniDrive` / `Gimbal` / `BodyAttitude` / `CommandWatchdog`）をファームと同じ順に通す。rabcl 系の規則に従い、ユニットテストと CI を持つ。
- 実装は C++（launch のみ `.launch.py`）。

非ゴール（今回やらない）:

- 物理シミュレーション（Gazebo 等）、車体が床の上を移動する様子（odom / TF）。関節の動きだけ。
- ros2_control / hardware_interface への移行。制御ループはマイコン側にあり、PC は高い抽象度の reference を渡すだけなので構造が合わない（将来 Gazebo をやるときに再検討）。
- モータのダイナミクス（遅れ・振動）、DM のイネーブル待ち、CAN の再現。
- beta の偽ファーム（beta を新プロトコルに移したあとで同じ土台に足す）。
- gamma の正確な寸法・メッシュ。CAD が見られないので仮の形で作り、帰宅後に差し替える。

アンチゴール（これが起きたら失敗）:

- 偽ファームのために rabbits_bridge に機体固有のコードや分岐が入る。
- rabcl に機体の値（gamma の定数）が入る。
- 偽ファームの変換がファームと違う式になり、「rviz では正しいのに実機で違う」が起きる。定数はファームの `robot.hpp` と対で更新し、式は rabcl の同じ部品を使う。

## 3. 構成

```
[gamepad_node / game_client_node]  rabbits_interface
          │ reference_command (rabbits_msgs/ReferenceCommand)
          ▼
[robot_bridge]  rabbits_bridge、config/gamma.yaml、device = /tmp/gamma_fake（pty へのシンボリックリンク）
          │ rabcl 共通プロトコル（CHASSIS_VEL / BODY_POSE / JOINT_CMD / MODE ↓、INFO / JOINT_STATE / SYSTEM ↑）
          ▼ pty（疑似端末。ソフトだけで作る仮想シリアルの組）
[gamma_fake_node]  rabbits_sim。ファームの変換を rabcl の部品で再現し、関節角を JOINT_STATE で返す
          ▲
          └─ joint_states は robot_bridge が出す（実機と同じ）
                    │
                    ▼
[robot_state_publisher（gamma の URDF）] ─ TF ─▶ [rviz2]   rabbits_description、launch は rabbits_bringup
```

実機のときは `robot_bridge` の `device` を `/dev/ttyUSB0`（USB-UART）や `/dev/ttyTHS1`（Jetson）にするだけで、上の interface と下の description は同じ。

## 4. rabbits_sim

ROS 2 パッケージ（ament_cmake、C++17）。依存: rclcpp、rabcl。rabcl_ros2 と rabbits_msgs は使わない（ROS のメッセージを扱わず、プロトコルのバイト列だけを扱う）。

### 4.1 部品

**`PtyLink`**（`include/rabbits_sim/pty_link.hpp`、`src/pty_link.cpp`）: 疑似端末の組を作り、こちら側（master）を読み書きする。

- `esp_err` のような独自型は使わず `bool Open(const std::string & link_path)` で `openpty()` → slave を raw モードにし、slave のパス（`/dev/pts/N`）へのシンボリックリンクを `link_path` に作る（既にあれば置き換える）。デストラクタでリンクを消して fd を閉じる。
- `int Read(uint8_t * buf, size_t len)`（ノンブロッキング。無ければ 0）、`int Write(const uint8_t * buf, size_t len)`。
- `std::string slave_path() const`。起動ログに出す。

**`FakeFirmware`**（`include/rabbits_sim/fake_firmware.hpp`、`src/fake_firmware.cpp`）: 機体に依らないプロトコル処理。ファームの `Comm` の PC 側版。

- 入力: `void Feed(const uint8_t * buf, size_t len)` → `rabcl::protocol::Parser` でフレームに分け、`CHASSIS_VEL` / `BODY_POSE` / `JOINT_CMD` / `MODE` を復号して `RobotModel`（下）に渡す。復号できたフレームの時刻を `RobotModel::OnCommandReceived(now_s)` で知らせる（途絶監視用）。
- 出力: `size_t Tick(double now_s, uint8_t * out, size_t cap)` → 100 Hz で呼ばれる想定。毎回 `JOINT_STATE`（model の全関節）、1 秒ごとに `INFO` と `SYSTEM`（`uptime_ms`、model の `error_flags`）を `out` に書き、長さを返す。
- pty も ROS も知らない。テストはバイト列で行う。

**`RobotModel`**（抽象、`include/rabbits_sim/robot_model.hpp`）: 機体ごとの振る舞い。

```cpp
class RobotModel {
public:
  virtual ~RobotModel() = default;
  virtual uint8_t joint_count() const = 0;
  virtual const char * firmware_id() const = 0;   // 8 文字以内
  virtual void SetChassisVel(const rabcl::protocol::ChassisVel &) = 0;
  virtual void SetBodyPose(const rabcl::protocol::BodyPose &) = 0;
  virtual void SetJointCmd(const rabcl::protocol::JointCmd &) = 0;
  virtual void SetMode(const rabcl::protocol::ModeCmd &) = 0;
  virtual void OnCommandReceived(double now_s) = 0;
  virtual void Update(double now_s, double dt) = 0;       // 関節の状態を dt だけ進める
  virtual void GetJointStates(rabcl::protocol::JointState * out) const = 0;  // joint_count 個
  virtual uint32_t error_flags() const = 0;
};
```

**`GammaModel`**（`include/rabbits_sim/gamma_model.hpp`、`src/gamma_model.cpp`）: gamma の制御の組み立てを再現する。設定は `GammaConfig`（構造体。値は YAML から読む。4.3）。

| 関節 | 再現する変換 | 関節角の決め方 |
|---|---|---|
| 脚 4（`LEG_FR, FL, BR, BL`、関節番号 7〜10） | `BodyPosture::Solve(height, roll, pitch, yaw_angle)` → 目標角、`LegDamper::Update` のランプ（`ramp_rate`） | ランプ後の目標角そのまま（モータは即応答。kp / kd / tff は使わない） |
| 車輪 4（関節番号 3〜6） | `CHASSIS_VEL` を `vel_max / rot_max` でクランプ、`chassis = 1` なら `wz += spin_speed`、`OmniDrive::CalcVel(vx, vy, wz, …, yaw_angle)`、`wheel_sign` | 車輪の回転速度 [rad/s] を dt で積分して角度にする（`continuous` 関節。±π に wrap） |
| 首 Yaw（関節番号 0） | `BodyAttitude`（IMU 無しなので `UpdateFromChassisRate(wz, dt)` で yaw を積分）、`Gimbal::Solve({yaw_cmd, 0, 0}, body)` | 関節角 = `Gimbal` の yaw そのまま（PD と LK トルクは再現しない） |
| 首 Pitch / Roll（関節番号 1, 2） | ファームは機構決定まで指令を送らない | 0 で固定（設定 `follow_neck_pitch_roll: true` にすると `JOINT_CMD` の pitch / roll をそのまま関節角にする。reference の確認用。既定は false） |

- 途絶（`CommandWatchdog`、`watchdog_s`）: 車輪 0、無限回転解除、首は最後の目標を保持、脚は最後に受けた `BODY_POSE` の姿勢へランプを進め切ってから止まる（ファームの `BuildLeg` と同じ。途絶の瞬間にしゃがみ途中なら、しゃがみ切る）。`error_flags` bit5。
- `error_flags` bit4（IMU 無効）は常に立てる（ファームも IMU ドライバ実装までは立つ）。bit0〜3（脚のトリップ）は再現しない（常に 0）。
- 起動時: 脚は `angle_min`（原点 = 最低車高。実機も原点リセットで 0 付近から始まる）、最初の `BODY_POSE` からランプで動く。首 Yaw は `JOINT_CMD` が来るまで 0。
- `JOINT_STATE` の `effort` / `temperature` は 0、`error` は 0。

**`gamma_fake_node`**（`src/gamma_fake_node.cpp`）: rclcpp ノード。パラメータ `link_path`（既定 `/tmp/gamma_fake`）、`tick_hz`（既定 100）、および gamma の定数（4.3 の `leg.*` / `wheel.*` / `chassis.*` / `watchdog_s` / `follow_neck_pitch_roll`。ROS の通常の params ファイルとして launch の `parameters=[...]` で渡す。C++ 側に機体の既定値は置かず、定数が未設定ならエラーログを出して終了する）。`PtyLink` を開き、wall timer で `Read → FakeFirmware::Feed`、`GammaModel::Update`、`FakeFirmware::Tick → Write`。ROS のトピックは持たない（ログだけ）。起動時に slave のパスとリンクを INFO で出す。

### 4.2 テスト（gtest、CI は rabcl 系と同じ ubuntu-24.04 + ros:jazzy-ros-base）

- `test_pty_link.cpp`: 開いてリンクができる、リンク先を `open()` して書いた 5 バイトが master から読める、逆向きも読める、閉じるとリンクが消える。
- `test_fake_firmware.cpp`: `Tick` が `JOINT_STATE`（joint_count 個）を出す、1 秒ごとに `INFO`（version、joint_count、firmware_id）と `SYSTEM` が出る、`CHASSIS_VEL` のフレームを `Feed` すると model に届く、CRC 不正は届かない。model はテスト用の記録クラス。
- `test_gamma_model.cpp`: 車高 0 / 1 / 0.5 で脚 4 本が `BodyPosture` と同じ角度（rabcl のテストと同じ値）に **ランプで** 到達する、ロール正で右脚が縮む、首 Yaw が 90° のとき BODY_POSE の pitch が車体の左右差になる（カメラ座標系）、`CHASSIS_VEL` vx > 0 で車輪角が時間に比例して進み `wheel_sign` の向き、`chassis = 1` で停止指令でも車輪が回る、途絶 0.5 s で車輪が止まり `error_flags` bit5 が立ち、途絶直前の `BODY_POSE` へのランプは進み切る、`JOINT_CMD` yaw で首 Yaw がその角になる、無限回転中に首 Yaw が胴体の回転を打ち消す向きに回る（`BodyAttitude` の積分）。

### 4.3 設定 `config/gamma.yaml`

ファームの `src/robot.hpp` と **対で更新**する（README に明記）。

```yaml
gamma_fake_node:
  ros__parameters:
    leg:
      angle_min: 0.15
      angle_max: 1.45
      tilt_gain: 1.0
      azimuth: [-0.7854, 0.7854, -2.3562, 2.3562]   # FR, FL, BR, BL
      ramp_rate: 0.5
    wheel:
      wheel_d: 0.06
      body_d: 0.28
      sign: -1.0
    chassis:
      vel_max: 1.0
      rot_max: 3.0
      spin_speed: 5.0
    watchdog_s: 0.5
    follow_neck_pitch_roll: false
```

## 5. rabbits_description（gamma の URDF）

- `urdf/gamma/gamma.xacro`（最上位）: 既存の proto1 と同じく部品マクロで組む。配置は機体ごと（`urdf/<機体>/`、`meshes/<機体>/`）、共有は `urdf/common/`（ユーザー決定 2026-10-24）。関節名は rabbits_bridge `config/gamma.yaml` の 11 個と **完全に一致**させる: `yaw_joint`, `pitch_joint`, `roll_joint`, `right_front_wheel_joint`, `left_front_wheel_joint`, `right_back_wheel_joint`, `left_back_wheel_joint`, `right_front_leg_joint`, `left_front_leg_joint`, `right_back_leg_joint`, `left_back_leg_joint`。
- リンク構成: `base_link` → 脚 4（`*_leg_link`、`revolute`、取り付け位置は方位角 FR −45° / FL 45° / BR −135° / BL 135°、半径 `BODY_D / 2 = 0.14 m`、回転軸は半径方向に直交する水平軸、範囲 0〜1.6 rad。角度 0 が最下点で、角度が増えると車体が上がる向き）→ 各脚の先に車輪（`continuous`、直径 0.06 m）。
- 車輪の関節は **車輪リンクの z 軸を回転軸**にし、**z 軸正が機体の外側**を向くように置く（ユーザー指定。モータの出力軸が外を向く取り付け）。`OmniDrive` の出力の符号と `wheel_sign` の意味はこの軸の向きに対して定義される。`base_link` → `yaw_link`（`continuous`）→ `pitch_link`（`revolute`、−π/12〜π/6）→ `roll_link`（`revolute`、±π/6）→ `camera_link`（固定。IMU とカメラの位置）。
- 寸法は仮（箱と円筒。胴体 0.28 m 角、脚の腕 0.10 m）。CAD を見られる帰宅後にメッシュと寸法を差し替える。軸の向きは上記の規約で固定し、寸法だけが仮であることを README と xacro のコメントに書く。rviz では `base_link` が固定なので、車高を上げると車輪が下に伸びて見える（床は描かない）。
- `launch/display.launch.py` に引数 `robot`（`proto1` | `gamma`、既定 `proto1`）を足し、xacro の選択に使う。rviz の設定は共通。
- README に gamma の節（関節名の出所、仮寸法の旨）。

## 6. rabbits_bringup（横断 launch）

ROS 2 パッケージ（ament_cmake、launch と設定だけ。C++ なし）。exec_depend に rabbits_sim、rabbits_bridge、rabbits_description、rabbits_interface、robot_state_publisher、rviz2、xacro。

- `launch/gamma_sim.launch.py`: `gamma_fake_node`（rabbits_sim）→ `robot_bridge`（`device:=/tmp/gamma_fake`、rabbits_bridge の `config/gamma.yaml`）→ `robot_state_publisher`（gamma.xacro）→ rviz2（`rviz/gamma.rviz`）→ 操作ノード（引数 `input`: `gamepad` | `game_client`、既定 `gamepad`。gamepad のときは joy_linux も）。bridge は偽ファームがリンクを作ってから起動する（`TimerAction` で 1 秒遅らせる。順序の簡単な解決。私の判断）。
- `launch/gamma.launch.py`: 実機用。`robot_bridge`（`device` 引数、既定 `/dev/ttyUSB0`）+ 操作ノード。rviz は引数 `rviz:=true` で任意。
- `rviz/gamma.rviz`: RobotModel と TF。
- `gamma.repos`: 上の依存パッケージの main（TODO にある「コミット固定」は実機で動いた時点で `vcs export --exact`）。
- CI: colcon build のみ（launch の構文は `ros2 launch rabbits_bringup gamma_sim.launch.py --show-args` 相当のチェックを入れられればよいが必須ではない）。

## 7. 設計者の判断（ユーザー未確認。違っていたら直す）

- モータは指令に即応答する（ダイナミクスなし）。脚のランプ（`LegDamper`）だけが時間変化を作る。実機のかたさ・遅れは見ない前提。
- 首 Pitch / Roll は既定 0 固定（ファームに忠実）。`follow_neck_pitch_roll` で reference の確認もできるようにする。
- DM のイネーブル待ちと脚のトリップは再現しない。`error_flags` は bit4（IMU 無効）と bit5（途絶）だけ。
- `firmware_id` は `"gammasm"`（7 文字。`Info.firmware_id[8]` を末尾 0 埋めにするため 8 文字は不可）。bridge のログで実機と区別できる。
- 偽ファームの更新と送信は 100 Hz（ファームの `Comm` の送信周期と同じ。CAN の 250 / 500 Hz は再現しない）。
- pty のリンクは `/tmp/gamma_fake`。bridge は 1 秒遅れで起動。pty の master はノンブロッキングにし、bridge が slave を開く前の書き込みは捨て、bridge を再起動しても偽ファームは生き続ける（偽ファーム側を再起動したら launch 全体を再起動する運用）。
- 偽ファームの定数は params ファイルのみで渡す（ノードに `config` パラメータは持たない）。未設定ならエラーで終了。
- 脚の初期角は `angle_min`。時刻は `std::chrono::steady_clock`、`dt` はファームと同じ 0.05 s で上限クランプ。`JOINT_STATE` の `velocity` は車輪が角速度、脚はランプの差分速度、首は 0。
- gamma の定数は YAML に二重に持つ（rabcl に機体の値を書かないため）。ファームの `robot.hpp` と対で更新する旨を両方の README に書く。
- 偽ファームは機体ごとに `RobotModel` を実装する（gamma 専用の分岐を共通部に入れない）。
- URDF の寸法は仮。脚は「腕が回って車輪を上下させる」単純なリンクで表し、平行リンクの幾何は CAD 確認後に反映（ファームの `BodyPosture` の幾何反映と同時）。

## 7.1 ユーザーが決めたこと（2026-10-24）

- 途絶時の脚は、最後に受けた `BODY_POSE` へランプを進め切る（ファームの今の動きが正）。
- 車輪の関節は車輪リンクの z 軸を回転軸にし、z 軸正が機体の外側。脚は「角度 0 が最下点、増えると車体が上がる」向き。
- 実装は C++、launch だけ `.launch.py`。rabbits_sim と rabbits_bringup は別リポジトリ。ros2_control と物理シミュレーションは今回やらない。

## 8. 実装者に任せること

- `PtyLink` の実装詳細（`openpty` / `posix_openpt`、raw 設定、ノンブロッキング）。
- `GammaConfig` の読み込み（rclcpp のパラメータ `leg.angle_min` のように、params ファイルのネストをドット区切りで宣言して読む。全項目を必須にする）。
- URDF の色・inertial（表示だけなので省いてよい）。
- テストの細かい値（rabcl の `test_body_posture.cpp` と同じ数値を使うと照合しやすい）。

## 9. 進め方

1. rabbits_sim: `PtyLink` → `FakeFirmware` → `GammaModel` → ノード → README / CI（各段で RED → GREEN）。
2. rabbits_description: gamma.xacro と launch の `robot` 引数。
3. rabbits_bringup: `gamma_sim.launch.py` と rviz 設定。Dev Container で `ros2 launch rabbits_bringup gamma_sim.launch.py` を実行し、ゲームパッド（usbipd で WSL に渡す）で全関節が動くことを目で確認。
4. 実機（帰宅後）: 同じ launch の `device` を実機に替えて差を見る。
