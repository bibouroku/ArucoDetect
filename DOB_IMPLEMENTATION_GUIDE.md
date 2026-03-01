# DOB (Disturbance Observer) 前馈补偿实现指南

## 概述

本文档说明如何在 `drone_tracker.cpp` 中使用 DOB 观测风扰并通过前馈补偿来提升无人机的抗风性能。

---

## 1. 代码改进概览

### 新增成员变量（在 `drone_tracker.hpp` 中）

```cpp
// --- DOB 相关成员变量 ---
Eigen::Vector3d _last_vehicle_velocity;     // 上一帧的无人机速度（用于计算加速度）
double _vehicle_mass = 2.0;                 // 无人机质量 (kg)
double _hover_thrust_norm = 0.5;            // 悬停时的标准化推力 (0-1)
double _max_thrust_newton = 19.6;           // 最大推力 (牛顿)

// --- PD 控制器参数 ---
double _kp = 1.0;                           // 位置增益
double _kd = 2.0;                           // 速度增益

// --- 控制指令缓存 ---
double _current_normalized_thrust = 0.5;    // 当前标准化推力 (0-1)
```

### 新增函数

```cpp
void publish_full_trajectory_setpoint(float x, float y, float z,
                                      float vx, float vy, float vz,
                                      float ax, float ay, float az);
```

---

## 2. DOB 更新流程详解

在 `run_tracking_state()` 中，DOB 的完整工作流程如下：

### 步骤 1：计算当前加速度

```cpp
Eigen::Vector3d current_accel = (_vehicle_velocity_ned - _last_vehicle_velocity) / dt_control;
_last_vehicle_velocity = _vehicle_velocity_ned;
```

- **输入**：当前速度与上一帧速度（来自 PX4 里程计）
- **输出**：加速度估计（单位：m/s²）
- **注意**：这是一阶差分，可能有噪音，可考虑加低通滤波

### 步骤 2：获取旋转矩阵

```cpp
Eigen::Matrix3d R_body_to_earth = _vehicle_orientation.toRotationMatrix();
```

- **作用**：将推力向量从机体坐标系转换到地球坐标系（NED）
- **来源**：`_vehicle_orientation` 四元数来自里程计

### 步骤 3：获取推力

```cpp
double thrust_newton = _current_normalized_thrust * _max_thrust_newton;
```

- **问题**：`_current_normalized_thrust` 需要从外部反馈获取
- **解决方案**（见下文第 3 节）：
  1. 订阅 PX4 的 `ActuatorMotors` 消息获取实时推力
  2. 或者从控制指令反推

### 步骤 4：更新 DOB

```cpp
dob_->update(current_accel, R_body_to_earth, thrust_newton);
```

- **参数**：
  - `current_accel`：惯性系加速度 (m/s²)
  - `R_body_to_earth`：机体到惯性系旋转矩阵
  - `thrust_newton`：总推力 (牛顿)

- **内部工作**（论文公式 15）：
  ```
  f_e(k+1) = (1 - β) * f_e(k) + β * [m*a - g - R*u_f]
  ```
  其中 β = (dt * K) / m

### 步骤 5：获取干扰补偿

```cpp
Eigen::Vector3d disturbance_force = dob_->getDisturbanceForce();
Eigen::Vector3d disturbance_accel = disturbance_force / _vehicle_mass;
```

- **disturbance_force**：风等外界干扰产生的力 (牛顿)
- **disturbance_accel**：对应的加速度 (m/s²)

### 步骤 6：计算最终控制指令

```cpp
Eigen::Vector3d cmd_accel_nominal = _kp * pos_error + _kd * vel_error;
Eigen::Vector3d cmd_accel_final = cmd_accel_nominal - disturbance_accel;
```

- **原理**：用观测到的干扰加速度补偿标称控制，抵消风的影响
- **效果**：无人机能更好地跟踪目标，抗风性能提升

---

## 3. 关键问题：获取实时推力反馈

### 问题分析

当前代码中 `_current_normalized_thrust = 0.5`（悬停推力） 是固定值，这会影响 DOB 的准确性。

### 解决方案 A：订阅 PX4 的推力反馈（推荐）

添加以下代码到 `drone_tracker.hpp` 的构造函数中：

```cpp
// 订阅 ActuatorMotors 消息获取实时推力
auto actuator_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
actuator_motors_sub_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>(
    "/fmu/out/actuator_motors",
    actuator_qos,
    [this](const px4_msgs::msg::ActuatorMotors::SharedPtr msg) {
        // 取四个电机推力的平均值，并标准化
        double avg_control = (msg->control[0] + msg->control[1] + 
                              msg->control[2] + msg->control[3]) / 4.0;
        _current_normalized_thrust = std::clamp(avg_control, 0.0, 1.0);
    }
);
```

并在 `drone_tracker.hpp` 中添加：

```cpp
rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_sub_;
```

### 解决方案 B：从控制指令反推

如果无法获取直接反馈，可以从发送的加速度指令反推：

```cpp
// 记录本次发送的指令
_current_normalized_thrust = std::clamp(
    (_hover_thrust_norm * _vehicle_mass * 9.81 + cmd_accel_final.z() * _vehicle_mass) 
    / _max_thrust_newton, 
    0.0, 1.0
);
```

### 解决方案 C：使用卡尔曼滤波反推推力

从加速度测量反推：

```cpp
// 简单版本：从加速度和姿态反推推力
double thrust_from_z_accel = (current_accel.z() + 9.81) * _vehicle_mass;
_current_normalized_thrust = std::clamp(thrust_from_z_accel / _max_thrust_newton, 0.0, 1.0);
```

---

## 4. 参数调整指南

### 4.1 质量参数

在启动参数中配置无人机质量：

```bash
# 在 launch 文件或命令行中设置
ros2 launch tracktor_beam drone_tracker.launch.py drone_mass:=2.5
```

或在 ROS 参数文件中：

```yaml
drone_tracker_controller:
  ros__parameters:
    drone:
      mass: 2.5              # 单位：kg
      hover_thrust_norm: 0.5 # 悬停时的标准化推力
    control:
      kp: 1.0                # 位置增益
      kd: 2.0                # 速度增益
    kf:
      q_std: 0.3
      r_std: 0.01
      dt: 0.03
```

### 4.2 DOB 增益调整

DOB 的效果由增益参数 K 控制（在 `dob_` 初始化中）：

```cpp
dob_ = std::make_unique<DisturbanceObserver>(
    _vehicle_mass,  // mass
    2.0,            // K（增益，通常 1.0-5.0）
    0.03            // dt（控制周期）
);
```

**K 的含义**：
- K 越大 → DOB 反应越快，但可能震荡
- K 越小 → DOB 更稳定，但反应较慢
- **推荐值**：2.0-3.0

### 4.3 PD 控制器参数

```cpp
_kp = 1.0;  // 位置增益：影响定位速度，过大会震荡
_kd = 2.0;  // 速度增益：阻尼效果，稳定性
```

调整建议：
- 如果跟踪误差大 → 增大 kp（但避免超 2.0）
- 如果响应震荡 → 增大 kd（阻尼）

---

## 5. 日志输出解读

运行时你会看到以下日志：

```
[INFO] [drone_tracker_controller]: [DOB INFO] Wind_Est: [0.234, -0.156] N, Accel_Est: [0.117, -0.078] m/s², CmdAcc: [1.234, -0.845] m/s², PosErr: [0.123, -0.456] m
```

**含义**：
- **Wind_Est**：估计的风力 (单位：牛顿)
- **Accel_Est**：风产生的加速度 (单位：m/s²)
- **CmdAcc**：最终发送给 PX4 的指令加速度（包含 DOB 补偿）
- **PosErr**：相对目标的位置误差

**判断 DOB 效果**：
- 如果 Wind_Est 长期保持小值（< 0.5N），说明风不大或算法正常
- 如果有风时 CmdAcc 与 Wind_Est 相反，说明 DOB 在正确补偿
- 如果跟踪误差减小，说明补偿生效

---

## 6. 完整流程示意

```
┌─────────────────────────────────────────────────────────────┐
│                   run_tracking_state()                       │
└─────────────────────────────────────────────────────────────┘
                              │
                ┌─────────────┴─────────────┐
                │                           │
        ┌───────▼────────┐         ┌───────▼──────────┐
        │  KF Filter     │         │  DOB Observer    │
        │  (Position)    │         │  (Disturbance)   │
        └───────┬────────┘         └───────┬──────────┘
                │                          │
         target_pos,                disturbance_
         target_vel                 accel
                │                          │
                └───────────┬──────────────┘
                            │
                ┌───────────▼────────────┐
                │  PD Control           │
                │  cmd_accel_nominal    │
                └───────────┬────────────┘
                            │
                ┌───────────▼────────────┐
                │  DOB Compensation     │
                │ - disturbance_accel   │
                └───────────┬────────────┘
                            │
                ┌───────────▼─────────────┐
                │ cmd_accel_final        │
                │ (抗风补偿后的指令)      │
                └───────────┬─────────────┘
                            │
                ┌───────────▼──────────────────┐
                │ publish_full_trajectory_     │
                │ setpoint(pos, vel, accel)    │
                └───────────┬──────────────────┘
                            │
                        发送给 PX4
```

---

## 7. 测试建议

### 测试场景 1：无风环境

1. 启动无人机和 drone_tracker
2. 让小车做匀速运动（v ≈ 0.5 m/s）
3. 观察日志中 Wind_Est 是否接近 0
4. 确认跟踪误差稳定在 0.1-0.2m 内

### 测试场景 2：有风环境

1. 在无人机周围设置风源（风扇）
2. 观察 Wind_Est 是否能反映风向和强度
3. 对比有/无 DOB 补偿的跟踪误差差异
4. 预期效果：有 DOB 时误差更小

### 测试场景 3：参数调优

1. 固定其他参数，只改变 DOB K 值
2. K = 1.0, 2.0, 3.0, 5.0 分别测试
3. 记录跟踪误差和响应速度
4. 选择最优的 K 值

---

## 8. 常见问题

**Q1: 为什么 Wind_Est 很大但无人机跟踪还可以？**

A: 可能是：
1. DOB 正在快速学习，补偿有效
2. PX4 内部位置控制已经很强，弥补了一部分误差
3. 建议增大 K 值让 DOB 反应更快

**Q2: Accel_Est 方向与实际风向相反？**

A: 这是正常的。因为：
- DOB 估计的是干扰加速度
- 最后我们做 cmd_accel_final = cmd_accel_nominal - disturbance_accel
- 所以如果真的有逆风，DOB 会估计出正向加速度来补偿

**Q3: 跟踪误差没有改善？**

A: 排查步骤：
1. 检查 _vehicle_mass 是否正确（日志中应该看到大的 Wind_Est 变化）
2. 检查推力反馈是否正确获取
3. 尝试增大 K 或 kp 值
4. 确认 PX4 收到了加速度指令（检查 msg.acceleration 字段是否非零）

---

## 9. 参考

- 论文公式 15：DOB 动力学方程
- PX4 文档：ActuatorMotors, VehicleOdometry
- Eigen 文档：Matrix3d, Vector3d 变换

