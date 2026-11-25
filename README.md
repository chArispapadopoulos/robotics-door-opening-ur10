# Robotics Project 2025 – Door Opening with a UR10

MATLAB code for the Robotics course project (ECE, AUTH). A door is opened in simulation in two steps. First the motion of the door handle is planned. Then a UR10 arm follows that motion with its end-effector.

## Requirements

- MATLAB
- [Robotics Toolbox for MATLAB](https://petercorke.com/toolboxes/robotics-toolbox/) by Peter Corke (uses `SerialLink`, `UnitQuaternion`, `trplot`, `transl`, `trotx/y/z`, `tr2delta`)

## Structure

```
.
├── codes/
│   ├── PartA/
│   │   └── door_motion.m               # handle/door trajectory and animation
│   └── PartB/
│       ├── save_handle_trajectory.m    # Part A + end-effector frame {E}, saves handle_traj.mat
│       └── ur10_door_opening.m         # UR10 follows the saved trajectory
└── Robotics_Report_EN.pdf              # project report (English translation)
```

## What each script does

**`PartA/door_motion.m`**
- Simulates 5 s with a step of 0.05 s, split into three phases:
  - 0–1.5 s: the handle turns to −45° about `x_H`.
  - 1.5–4 s: the door opens to −30° about `z_D`.
  - 4–5 s: the handle returns to 0°.
- Every phase uses a quintic polynomial, so velocity and acceleration are zero at both ends.
- Builds the door frame `{D}` and the handle frame `{H}` with homogeneous transformations and animates the door and the frames in 3D.
- Plots the unit quaternion of `{H}` over time.

**`PartB/save_handle_trajectory.m`**
- Same motion as Part A, plus the frame `{E}` of the gripper holding the handle: `T_E = T_H * T_EH * T_align`.
- Stores the pose of `{E}` at every time step and saves `handle_traj.mat` (`t`, `T_E_log`).
- Plots the quaternion of `{E}`.

**`PartB/ur10_door_opening.m`**
- Loads `handle_traj.mat` and builds a UR10 model (DH parameters, base at `[1 1 0]`).
- At every time step, it solves velocity-level inverse kinematics with the pseudo-inverse of the Jacobian. It runs up to 5 iterations and stops earlier when the pose error drops below 1e-3.
- Animates the robot opening the door.
- Plots joint positions and joint velocities (finite differences).

## How to run

1. Part A: run `codes/PartA/door_motion.m`.
2. Part B:
   1. Run `codes/PartB/save_handle_trajectory.m` first. It creates `handle_traj.mat`.
   2. Then run `codes/PartB/ur10_door_opening.m` from the same folder.