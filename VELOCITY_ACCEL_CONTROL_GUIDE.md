# 速度+加速度控制模式下的定高悬停实现方案

## 当前代码分析

你的代码已经有了一个 `HOLDING` 状态：

```cpp
void DroneTrackerController::run_holding_state()
{
    publish_offboard_control_mode();
    RCLCPP_INFO(this->get_logger(), "State: HOLDING");
    if (!hold_inited_) {
        hold_start_time_ = this->now();
        hold_inited_ = true;
    }
    publish_trajectory_setpoint(0,0,HEIGHT);  // 位置控制
    const double t = (this->now() - hold_start_time_).seconds();
    if (t >= hold_duration_sec_) {
        switchToState(State::TRACKING);
    }
}
```

## 问题所在

现在你改成了**速度+加速度控制**，所以：
- ❌ 不能再用 `publish_trajectory_setpoint(0, 0, HEIGHT)` 发送位置
- ✅ 需要发送速度=0（悬停）+ 加速度=0（不加力）

## 解决方案

### 核心思路

在悬停阶段，你需要：

1. **Z轴高度维持**：发送零速度（悬停）+ 零加速度（不再加力）
2. **XY轴锁定**：也发送零速度 + 零加速度（保持静止）

这样 PX4 的内部位置环会自动维持高度。

### 修改 OffboardControlMode

首先确保你的控制模式设置正确：

```cpp
void DroneTrackerController::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;      // ❌ 禁用位置控制
    msg.velocity = true;       // ✅ 启用速度控制
    msg.acceleration = true;   // ✅ 启用加速度反馈
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}
```

### 修改 HOLDING 状态

```cpp
void DroneTrackerController::run_holding_state()
{
    publish_offboard_control_mode();
    
    if (!hold_inited_) {
        hold_start_time_ = this->now();
        hold_inited_ = true;
        RCLCPP_INFO(this->get_logger(), "Entering HOLDING state, will hold for %.1f seconds", 
                    hold_duration_sec_);
    }

    // 关键：发送零速度悬停（不发送位置）
    // velocity = [0, 0, 0] 意思是：保持当前高度
    // acceleration = [0, 0, 0] 意思是：不施加额外的加速度
    publish_velocity_setpoint(0.0, 0.0, 0.0,      // vx, vy, vz = 0
                             0.0, 0.0, 0.0);      // ax, ay, az = 0
    
    const double t = (this->now() - hold_start_time_).seconds();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "HOLDING: %.1f / %.1f seconds",
                        t, hold_duration_sec_);
    
    if (t >= hold_duration_sec_) {
        RCLCPP_INFO(this->get_logger(), "Hold duration reached, switching to TRACKING");
        hold_inited_ = false;  // 重置标志，准备下次使用
        switchToState(State::TRACKING);
    }
}
```

### 新增辅助函数

在 `drone_tracker.hpp` 中添加声明：

```cpp
// 在 public 方法中添加
void publish_velocity_setpoint(float vx, float vy, float vz,
                               float ax, float ay, float az);
```

在 `drone_tracker.cpp` 中实现：

```cpp
void DroneTrackerController::publish_velocity_setpoint(float vx, float vy, float vz,
                                                       float ax, float ay, float az)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // 只用速度和加速度，位置设为 NaN
    const float NaN = std::numeric_limits<float>::quiet_NaN();
    msg.position = {NaN, NaN, NaN};

    // 发送速度和加速度
    msg.velocity = {vx, vy, vz};
    msg.acceleration = {ax, ay, az};

    msg.yaw = NaN;

    trajectory_setpoint_publisher_->publish(msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                "Published Velocity Setpoint - Vel:[%.3f, %.3f, %.3f], Accel:[%.3f, %.3f, %.3f]",
                vx, vy, vz, ax, ay, az);
}
```

### 修改 TRACKING 状态

在跟踪时，改为发送速度+加速度而不是位置+加速度：

```cpp
void DroneTrackerController::run_tracking_state()
{
    publish_offboard_control_mode();
    // ... 其他代码 ...
    
    // 计算目标速度（来自卡尔曼滤波或速度估计）
    Eigen::Vector3d target_vel = kf_filter_->getFilteredVelocity();
    
    // 计算速度误差并进行反馈
    Eigen::Vector3d vel_error = target_vel - _vehicle_velocity_ned;
    
    // 速度控制：用速度误差 + 加速度前馈
    // 加速度前馈 = DOB 补偿 + 位置反馈转换的加速度
    Eigen::Vector3d pos_error = target_pos - _vehicle_position_ned;
    Eigen::Vector3d accel_from_pos = _kp * pos_error;  // 位置误差转加速度
    
    // 最终发送的加速度 = 位置反馈 + DOB补偿
    Eigen::Vector3d cmd_accel_final = accel_from_pos + disturbance_accel;
    cmd_accel_final.z() = 0.0;
    
    // 发送速度设定点（来自卡尔曼滤波器的目标速度）
    publish_velocity_setpoint(
        target_vel.x(), target_vel.y(), 0.0,           // 目标速度（Z=0，高度锁定）
        cmd_accel_final.x(), cmd_accel_final.y(), 0.0  // 加速度补偿
    );
}
```

## 关键点总结

| 阶段 | Position | Velocity | Acceleration |
|------|----------|----------|--------------|
| **ARMING** | 位置 (0,0,H) | NaN | NaN |
| **HOLDING** | NaN | 0,0,0 | 0,0,0 |
| **TRACKING** | NaN | Kalman速度 | PD+DOB |

## 工作原理

1. **Arming → Holding**：
   - PX4 接收到零速度指令
   - 内部位置环自动维持高度
   - 无人机悬停在当前高度

2. **Holding → Tracking**：
   - 开始发送目标速度（来自卡尔曼滤波）
   - 同时发送加速度补偿（位置误差 + DOB）
   - 无人机开始水平移动，高度保持不变

## 参数调整建议

```yaml
# config_example_dob.yaml
drone_tracker_controller:
  ros__parameters:
    # 悬停参数
    hold_duration: 3.0        # 悬停 3 秒
    
    # 速度控制参数
    control:
      kp: 1.5      # 位置到加速度的转换增益（用于跟踪时的加速度反馈）
      kd: 0.0      # 在速度模式下通常设为 0（不直接用速度增益）
```

## 测试步骤

1. **启动节点**，观察状态转换：
   ```
   [State transition] IDLE → ARMING → HOLDING → TRACKING
   ```

2. **在 HOLDING 阶段**，检查日志：
   ```
   [HOLDING] Vel=[0.000, 0.000, 0.000] Accel=[0.000, 0.000, 0.000]
   ```

3. **进入 TRACKING 阶段**，观察速度指令变化：
   ```
   [TRACKING] Vel=[0.123, 0.456, 0.000] Accel=[0.05, 0.08, 0.000]
   ```

4. **检查高度**：
   - Z 轴应该保持不变（±0.1m以内）

## 常见问题

**Q: 为什么还要发送加速度，不能只用速度？**

A: 纯速度控制响应较慢（30Hz更新率，延迟较大）。加速度前馈可以提前补偿，让无人机响应更快。

**Q: 位置误差怎样转换成加速度？**

A: 用简单的比例关系：`accel = kp × position_error`。这相当于把位置环外移到应用层。

**Q: Z轴为什么要设为0？**

A: 因为你想"定高悬停"，Z速度=0意味着保持当前高度。加速度=0表示不要额外升降。

## 完整状态机流程

```
[IDLE]
   ↓
[ARMING] → 发送位置悬停指令 (0, 0, HEIGHT)
   ↓ (收到 Offboard 模式确认)
[HOLDING] → 发送零速度 (0, 0, 0) + 零加速度 (0, 0, 0)
   ↓ (等待 hold_duration 秒)
[TRACKING] → 发送目标速度 (vx, vy, 0) + 补偿加速度 (ax, ay, 0)
   ↓ (目标静止 5+ 秒且接近)
[DESCEND] → 发送 LAND 指令
```

这样就完美实现了：**先用位置悬停 → 再用速度跟踪** 的过渡过程！
