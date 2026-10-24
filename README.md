# rabbits_sim

[![CI](https://github.com/rabbits-robotics/rabbits_sim/actions/workflows/ci.yml/badge.svg)](https://github.com/rabbits-robotics/rabbits_sim/actions/workflows/ci.yml)

機体のファームの代わりに [rabcl](https://github.com/rabbits-robotics/rabcl) の共通プロトコルで応答する偽ファーム。実機なしで [rabbits_bridge](https://github.com/rabbits-robotics/rabbits_bridge) から先（`joint_states` → rviz）を動かし、PC 側が作る指令を目で確かめる。まず gamma。

設計書: [docs/superpowers/specs/2026-10-24-gamma-sim-design.md](docs/superpowers/specs/2026-10-24-gamma-sim-design.md)、計画: [docs/superpowers/plans/2026-10-24-gamma-sim.md](docs/superpowers/plans/2026-10-24-gamma-sim.md)。

## テスト

```bash
colcon build --packages-up-to rabbits_sim
colcon test --packages-select rabbits_sim
colcon test-result --verbose
```
