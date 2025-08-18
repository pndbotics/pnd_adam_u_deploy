# pnd_adam_u_deploy

## adam_u 类型选择

Change the variable `adam_type` in the file `src/RobotControl/public_parameter.cpp` to select whether to include the hand module

## Compilation

```shell
colcon build --packages-select pnd_adam_u_deploy --cmake-clean-cache
```

## Execution

```shell
sudo ./run.sh
```

> **Note**  
> The above command only starts the `pnd_adam_u_deploy` node.  
> To control `adam_u`, execute `/home/pnd-humanoid/Documents/adam_u_demo/run_pndrobotros2.sh`  
> to ensure the `pndrobotros2` node launches properly.

---

## Joint Data Acquisition

### 1. Subscribe to `/joint_cmd` (Command Data)

**Message Field Specifications**  

| Field | Dimensions | Description |
|-------------|------------|----------------------|
| `q_d` | 25 | Desired joint positions |
| `q_dot_d` | 25 | Desired joint velocities |
| `tau_d` | 25 | Desired joint torques |
| `hands_d` | 12 | Desired hand positions |

### 2. Subscribe to `/robot_state` (Actual State)

**Message Field Specifications**  

| Field | Dimensions | Description |
|-------------|------------|----------------------|
| `q_a` | 25 | Actual joint positions |
| `q_dot_a` | 25 | Actual joint velocities |
| `tau_a` | 25 | Actual joint torques |
| `hands_a` | 12 | Actual hand positions |

---

## Dimension Mapping Specifications

### Body Joints (25 Dimensions)

| Index | Joint Name             | Body Side |
| ----- | ---------------------- | --------- |
| 0-5   | Base data (fixed to 0) | -         |
| 6     | `waistRoll`            | Torso     |
| 7     | `waistPitch`           | Torso     |
| 8     | `waistYaw`             | Torso     |
| 9     | `neckYaw`              | Neck      |
| 10    | `neckPitch`            | Neck      |
| 11    | `shoulderPitch`        | Left Arm  |
| 12    | `shoulderRoll`         | Left Arm  |
| 13    | `shoulderYaw`          | Left Arm  |
| 14    | `elbow_Left`           | Left Arm  |
| 15    | `wristYaw`             | Left Arm  |
| 16    | `wristPitch`           | Left Arm  |
| 17    | `wristRoll`            | Left Arm  |
| 18    | `shoulderPitch`        | Right Arm |
| 19    | `shoulderRoll`         | Right Arm |
| 20    | `shoulderYaw`          | Right Arm |
| 21    | `elbow_Right`          | Right Arm |
| 22    | `wristYaw`             | Right Arm |
| 23    | `wristPitch`           | Right Arm |
| 24    | `wristRoll`            | Right Arm |

### Hand Data (12 Dimensions)

| Index Range | Corresponding Hand |
| ----------- | ------------------ |
| 0-5         | Left Hand          |
| 6-11        | Right Hand         |

**Per-Finger Joint Mapping**  

| Index | Finger Part |
|-------|---------------------------|
| 1 | Pinky |
| 2 | Ring Finger |
| 3 | Middle Finger |
| 4 | Index Finger |
| 5 | Thumb |
| 6 | Thumb Lateral Movement |
