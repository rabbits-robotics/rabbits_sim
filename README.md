# rabbits_sim

[![CI](https://github.com/rabbits-robotics/rabbits_sim/actions/workflows/ci.yml/badge.svg)](https://github.com/rabbits-robotics/rabbits_sim/actions/workflows/ci.yml)

機体のファームの代わりに [rabcl](https://github.com/rabbits-robotics/rabcl) の共通プロトコルで応答する偽ファーム。実機なしで [rabbits_bridge](https://github.com/rabbits-robotics/rabbits_bridge) から先（`joint_states` → rviz）を動かし、PC 側が作る指令を目で確かめる。まず gamma。

設計書: [docs/superpowers/specs/2026-10-24-gamma-sim-design.md](docs/superpowers/specs/2026-10-24-gamma-sim-design.md)、計画: [docs/superpowers/plans/2026-10-24-gamma-sim.md](docs/superpowers/plans/2026-10-24-gamma-sim.md)。

```
[gamepad_node / game_client_node]  rabbits_interface
          │ reference_command
          ▼
[robot_bridge]  rabbits_bridge、config/gamma.yaml、device:=/tmp/gamma_fake
          │ rabcl 共通プロトコル（CHASSIS_VEL / BODY_POSE / JOINT_CMD / MODE ↓、INFO / JOINT_STATE / SYSTEM ↑）
          ▼ pty（疑似端末。ソフトだけで作る仮想シリアルの組。/tmp/gamma_fake → /dev/pts/N）
[gamma_fake_node]  rabbits_sim
          ▲
          └─ joint_states は robot_bridge が出す（実機と同じ）→ robot_state_publisher（rabbits_description の gamma）→ rviz
```

実機のときは `robot_bridge` の `device` を実機のポートにするだけで、上下は同じ。横断 launch は [rabbits_bringup](https://github.com/rabbits-robotics/rabbits_bringup)。

## 部品

| ファイル | 役割 |
|---|---|
| `include/rabbits_sim/pty_link.hpp`, `src/pty_link.cpp` | `PtyLink`。pty の組を作り、slave へのシンボリックリンクを `link_path` に張る。master はノンブロッキング。slave の fd も持ち続けるので bridge が再起動しても生きる |
| `include/rabbits_sim/robot_model.hpp` | `RobotModel`。機体の振る舞いの口（純粋仮想）。機体を足すときはこれを実装する |
| `include/rabbits_sim/fake_firmware.hpp`, `src/fake_firmware.cpp` | `FakeFirmware`。ファームの `Comm` の PC 側版。下りフレームを `RobotModel` に渡し、`JOINT_STATE` を毎回、`INFO` / `SYSTEM` を 1 s ごとに書き出す |
| `include/rabbits_sim/gamma_model.hpp`, `src/gamma_model.cpp` | `GammaModel`。gamma のファーム（gamma_turret_fw の `Controller`）の変換を rabcl の同じ部品で再現: 車輪 = `OmniDrive`（首 Yaw で視点基準）+ 無限回転、脚 = `BodyPosture` → `LegDamper` のランプ、首 Yaw = `Gimbal`（視点目標 − `BodyAttitude`）。モータは即応答 |
| `src/gamma_fake_node.cpp` | `gamma_fake_node`。3 つをつなぐ。ROS トピックは持たない |
| `config/gamma.yaml` | gamma の定数。**gamma_turret_fw の `src/robot.hpp` と対で更新する** |
| `launch/gamma_fake.launch.py` | 偽ファームだけを起動（単体確認用） |

挙動（ファームと同じにしてある）: 途絶 0.5 s で車輪 0・無限回転解除、脚は最後の `BODY_POSE` へランプを進め切る、首 Yaw は最後の目標を保持。`error_flags` は IMU 無効（bit4、常時）と途絶（bit5）。首 Pitch / Roll はファームが機構決定まで送らないので 0（`follow_neck_pitch_roll: true` で `JOINT_CMD` に追従させて reference の確認ができる）。`firmware_id` は `gammasm`（7 文字。実機は `gamma`）。

## 使い方

```bash
ros2 launch rabbits_bringup gamma_sim.launch.py            # 通しで動かすときはこれ（偽ファーム + bridge + rviz + 操作）

# 偽ファームだけ、bridge だけを手で起動するとき（robot_bridge.launch.py には device 引数が無いので ros2 run で渡す）
ros2 launch rabbits_sim gamma_fake.launch.py               # /tmp/gamma_fake ができる
ros2 run rabbits_bridge robot_bridge_node --ros-args --params-file $(ros2 pkg prefix rabbits_bridge)/share/rabbits_bridge/config/gamma.yaml -p device:=/tmp/gamma_fake
ros2 topic echo /joint_states --once
```

**rviz での車輪の向きの読み方**: 車輪の角速度は `OmniDrive::CalcVel` の出力 × `wheel_sign` をそのまま積分している（ファームと同じ式）。URDF の規約（車輪の z 軸が機体の外側）に対して `OmniDrive` の並進項と回転項の符号関係が揃っていないため、現状の `wheel_sign = -1` では **前進指令で車輪が後ろに転がって見え、旋回は幾何どおりに見える**。これは偽ファームや URDF の不具合ではなく、実機で `WHEEL_SIGN` を確定したあとに rabcl の `OmniDrive` の符号（または URDF の軸向き）を揃えるタスクで解消する。それまでは「式どおりに回っているか」を見る。

パラメータ（`gamma_fake_node`）: `link_path`（既定 `/tmp/gamma_fake`）、`tick_hz`（既定 100）、`leg.*` / `wheel.*` / `chassis.*` / `watchdog_s`（必須。`config/gamma.yaml` で渡す。無ければエラーで終了）、`follow_neck_pitch_roll`（既定 false）。

偽ファームを再起動したら pty が変わるので bridge も再起動する（launch 全体の再起動）。bridge だけの再起動は偽ファームを止めずにできる。

## テスト

```bash
colcon build --packages-up-to rabbits_sim
colcon test --packages-select rabbits_sim
colcon test-result --verbose
```

`test_pty_link`（pty の往復）、`test_fake_firmware`（バイト列でプロトコル）、`test_gamma_model`（rabcl の `test_body_posture` と同じ数値で脚、`OmniDrive` と同じ式で車輪、途絶・無限回転・首）。
