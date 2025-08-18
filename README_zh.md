# pnd_adam_u_deploy

## adam_u 类型选择

> 在目录`src/RobotControl/public_parament.cpp`中修改参数`adam_type`即可选择是否带手
>
## 编译

```shell
colcon build --packages-select pnd_adam_u_deploy --cmake-clean-cache
```

## 运行

```shell
sudo ./run.sh
```

> **注意**  
> 上述命令仅启动 `pnd_adam_u_deploy` 节点。  
> 如需控制 `adam_u`，需执行 `/home/pnd-humanoid/Documents/adam_u_demo/run_pndrobotros2.sh`  
> 以确保 `pndrobotros2` 节点正常启动。

---

## 关节数据获取

### 1. 订阅 `/joint_cmd` (下发指令)

**消息字段说明**  

| 字段 | 维度 | 含义 |
|------------|------|------------------|
| `q_d` | 25 | 关节期望位置 |
| `q_dot_d` | 25 | 关节期望速度 |
| `tau_d` | 25 | 关节期望力矩 |
| `hands_d` | 12 | 手部期望位置 |

### 2. 订阅 `/robot_state` (实际状态)

**消息字段说明**  

| 字段 | 维度 | 含义 |
|------------|------|------------------|
| `q_a` | 25 | 关节实际位置 |
| `q_dot_a` | 25 | 关节实际速度 |
| `tau_a` | 25 | 关节实际力矩 |
| `hands_a` | 12 | 手部实际位置 |

---

## 维度映射说明

### 身体关节 (25维)

| 索引 | 关节名称           | 身体侧 |
| ---- | ------------------ | ------ |
| 0-5  | 基座数据 (固定为0) | -      |
| 6    | `waistRoll`        | 躯干   |
| 7    | `waistPitch`       | 躯干   |
| 8    | `waistYaw`         | 躯干   |
| 9    | `neckYaw`          | 颈部   |
| 10   | `neckPitch`        | 颈部   |
| 11   | `shoulderPitch`    | 左臂   |
| 12   | `shoulderRoll`     | 左臂   |
| 13   | `shoulderYaw`      | 左臂   |
| 14   | `elbow_Left`       | 左臂   |
| 15   | `wristYaw`         | 左臂   |
| 16   | `wristPitch`       | 左臂   |
| 17   | `wristRoll`        | 左臂   |
| 18   | `shoulderPitch`    | 右臂   |
| 19   | `shoulderRoll`     | 右臂   |
| 20   | `shoulderYaw`      | 右臂   |
| 21   | `elbow_Right`      | 右臂   |
| 22   | `wristYaw`         | 右臂   |
| 23   | `wristPitch`       | 右臂   |
| 24   | `wristRoll`        | 右臂   |

### 手部数据 (12维)

| 索引范围 | 对应手部 |
| -------- | -------- |
| 0-5      | 左手     |
| 6-11     | 右手     |

**单手指关节映射**  

| 索引 | 手指部位 |
|------|------------------|
| 1 | 小拇指 (Pinky) |
| 2 | 无名指 (Ring) |
| 3 | 中指 (Middle) |
| 4 | 食指 (Index) |
| 5 | 大拇指 (Thumb) |
| 6 | 大拇指侧向 (Thumb Lateral) |
