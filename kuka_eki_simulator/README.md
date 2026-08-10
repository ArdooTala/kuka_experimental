# KUKA Simulation

This package contains components for simulation of KUKA robots using the KUKA.Eki.

## Motion primitives simulator

`kuka_eki_motion_primitives_simulator` simulates the KRC side of the
`kuka_eki_motion_primitives_hw_interface` driver:

- TCP motion channel (port 54600): parses `<RobotCommand>` frames and answers
  with `<ProgramState>` frames (idle/executing/stopped, per-command
  started/finished/error) mirroring `krl/motion_eki.src`, including sequence
  handling and `<AbortCommands/>` / `<ResetAbortCommands/>`.
- UDP meta channel (port 54601): streams `<RobotState>` kinematics at ~100 Hz
  after the client registers with its first datagram.
- PTP joint motions are simulated with synchronized constant-velocity moves
  (no acceleration). Cartesian (LIN/CIRC) modes are reported as errors.

```sh
ros2 run kuka_eki_simulator kuka_eki_motion_primitives_simulator
```

Options: `--eki_hw_iface_ip`, `--eki_hw_iface_motion_port`,
`--eki_hw_iface_meta_port` (0 disables the meta channel).

The simulator logic lives in the `core/` subpackage (`Robot`, `E6Axis`,
`E6Pos`, `RobotCommand` parser, `ProgramState`/`RobotState` builders, EKI
servers, `MotionInterpreter`).

The legacy `kuka_eki_simulator` (UDP) and `kuka_eki_simulator_tcp` entry
points are kept untouched.
