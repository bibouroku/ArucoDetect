# DOB 系统架构与数据流

## 🏗️ 系统架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                     DroneTrackerController                       │
│                        (ROS2 Node)                               │
└─────────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
    ┌────────┐         ┌─────────────┐        ┌──────────┐
    │ Input  │         │   Process   │        │ Output   │
    │Channels│         │   Flow      │        │ Topics   │
    └────────┘         └─────────────┘        └──────────┘
        │                   │                     │
   [Measurement]        [Control Logic]       [Setpoints]
   [Odometry]           [DOB]                 [Trajectory]
```

## 📊 完整数据流

```
PX4 Firmware
    │
    ├─→ VehicleOdometry (30 Hz)
    │   ├─ position[x, y, z]
    │   ├─ velocity[vx, vy, vz]
    │   └─ attitude (四元数)
    │
    └─→ VehicleControlMode (检测 OFFBOARD 模式)

Camera/ArUco System (20 Hz)
    │
    └─→ PoseStamped (/target_pose)
        └─ position (目标位置)

        ▼
    ┌─────────────────────────────────┐
    │   DroneTrackerController        │
    │                                 │
    │  ┌──────────────────────────┐  │
    │  │   pose_callback()        │  │
    │  │ (处理测量值 ~20 Hz)       │  │
    │  └────────┬─────────────────┘  │
    │           │                    │
    │           ▼                    │
    │  ┌──────────────────────────┐  │
    │  │  KFTrackerCore           │  │
    │  │  (Kalman Filter)         │  │
    │  │  - 融合测量              │  │
    │  │  - 输出 [pos, vel]       │  │
    │  └────────┬─────────────────┘  │
    │           │                    │
    │           │ 滤波结果           │
    │           │ (已平滑速度)       │
    │           ▼                    │
    │  ┌──────────────────────────┐  │
    │  │ run_tracking_state()     │  │
    │  │ (主控制循环 ~30 Hz)      │  │
    │  │                          │  │
    │  │ ┌────────────────────┐  │  │
    │  │ │ 1. 读取状态        │  │  │
    │  │ │    - KF 位置       │  │  │
    │  │ │    - 里程计速度    │  │  │
    │  │ │    - 里程计姿态    │  │  │
    │  │ └────────┬───────────┘  │  │
    │  │          ▼              │  │
    │  │ ┌────────────────────┐  │  │
    │  │ │ 2. 计算加速度      │  │  │
    │  │ │  a = Δv / dt       │  │  │
    │  │ │    (速度微分)      │  │  │
    │  │ └────────┬───────────┘  │  │
    │  │          ▼              │  │
    │  │ ┌────────────────────┐  │  │
    │  │ │ 3. DOB 更新        │  │  │
    │  │ │ dob_->update(      │  │  │
    │  │ │   accel,           │  │  │
    │  │ │   R_body_to_earth, │  │  │
    │  │ │   thrust_newton    │  │  │
    │  │ │ )                  │  │  │
    │  │ │ ↓                  │  │  │
    │  │ │ 估计风力           │  │  │
    │  │ └────────┬───────────┘  │  │
    │  │          ▼              │  │
    │  │ ┌────────────────────┐  │  │
    │  │ │ 4. PD 控制         │  │  │
    │  │ │ u_pd = kp*err_p +  │  │  │
    │  │ │        kd*err_v    │  │  │
    │  │ └────────┬───────────┘  │  │
    │  │          ▼              │  │
    │  │ ┌────────────────────┐  │  │
    │  │ │ 5. DOB 补偿        │  │  │
    │  │ │ u_final =          │  │  │
    │  │ │ u_pd - d_hat       │  │  │
    │  │ │ (补偿风扰)         │  │  │
    │  │ └────────┬───────────┘  │  │
    │  │          ▼              │  │
    │  │ ┌────────────────────┐  │  │
    │  │ │ 6. 发送指令        │  │  │
    │  │ │ publish_full_      │  │  │
    │  │ │ trajectory_        │  │  │
    │  │ │ setpoint()         │  │  │
    │  │ └────────┬───────────┘  │  │
    │  │          │              │  │
    │  └──────────┼──────────────┘  │
    │             │                  │
    └─────────────┼──────────────────┘
                  │
    ┌─────────────▼──────────────┐
    │  TrajectorySetpoint        │
    │  /fmu/in/               │  │
    │  trajectory_setpoint    │  │
    │                         │  │
    │  • position[x,y,z]      │  │
    │  • velocity[vx,vy,vz]   │  │
    │  • acceleration         │  │
    │    [ax,ay,az]           │  │
    │                         │  │
    └─────────────┬──────────────┘
                  │ (PX4 FixedWing/MC)
                  │ Position + Velocity
                  │ + Acceleration
                  ▼ Control
                PX4 FW Controller
                 (内部循环)
                  │
                  ▼
            Motor Commands
```

## 🔄 控制流程详解

### 第一层：测量融合（20 Hz - Camera）

```
Raw Measurement (ArUco)
    │
    └─→ pose_callback()
        ├─ 坐标变换 (Camera → NED)
        └─ KFTrackerCore::updateWithMeasurement()
           │
           ├─ predict()
           │  └─ F = [I, dt*I; 0, I]
           │  └─ 状态预测
           │
           └─ update()
              └─ Kalman 增益更新
              └─ 输出平滑的位置和速度
```

### 第二层：主控制循环（30 Hz - Timer）

```
run_tracking_state()
    │
    ├─ Step 1: 状态读取
    │  ├─ filtered_position = KF.getFilteredState().position
    │  ├─ vehicle_velocity = odometry.velocity
    │  └─ vehicle_attitude = odometry.attitude
    │
    ├─ Step 2: 速度微分 (加速度计算)
    │  ├─ current_accel = (velocity - last_velocity) / dt
    │  ├─ last_velocity ← velocity
    │  └─ [输出] a_inertial (m/s²)
    │
    ├─ Step 3: 准备 DOB 输入
    │  ├─ R_body_to_earth = attitude.toRotationMatrix()
    │  ├─ thrust_newton = thrust_normalized * max_thrust
    │  └─ [输出] R (旋转矩阵), T (推力)
    │
    ├─ Step 4: DOB 风估计
    │  ├─ dob_->update(a_inertial, R, T)
    │  │  └─ 内部：fe(k+1) = (1-β)*fe(k) + β*[m*a - g - R*u]
    │  ├─ dist_force = dob_->getDisturbanceForce()
    │  ├─ dist_accel = dist_force / mass
    │  └─ [输出] 风的加速度影响
    │
    ├─ Step 5: PD 控制
    │  ├─ target_pos = filtered_pos + KF_vel * horizon
    │  ├─ target_vel = KF_vel
    │  ├─ pos_error = target_pos - vehicle_pos
    │  ├─ vel_error = target_vel - vehicle_vel
    │  ├─ cmd_accel_pd = kp * pos_error + kd * vel_error
    │  └─ [输出] u_pd
    │
    ├─ Step 6: 风补偿
    │  ├─ cmd_accel_final = cmd_accel_pd - dist_accel
    │  ├─ cmd_accel_final.z = 0  (仅 XY 平面)
    │  └─ [输出] u_final (补偿后指令)
    │
    └─ Step 7: 发送指令
       └─ publish_full_trajectory_setpoint(
              pos, vel, accel=cmd_accel_final
          )
          └─ msg.acceleration ← [ax, ay, 0]
             (PX4 将其作为加速度前馈)
```

## 🧮 数学模型

### DOB 核心公式（论文公式 15）

**系统动力学**：
$$m \ddot{p} = u + f_e - mg$$

其中：
- $m$ = 无人机质量
- $p$ = 位置
- $u$ = 控制推力向量
- $f_e$ = 干扰力（风）
- $g$ = 重力加速度

**DOB 估计**（低通滤波）：
$$\hat{f}_e(k+1) = (1-\beta)\hat{f}_e(k) + \beta[m\ddot{p}_{meas} - g - u]$$

其中：
- $\beta = \frac{dt \cdot K}{m}$
- $K$ = 观测器增益（通常 2-3）

**最终补偿**：
$$u_{final} = u_{nominal} - \frac{\hat{f}_e}{m}$$

### 坐标系约定

```
NED 坐标系（北东地）
    ├─ N（北）：x 轴
    ├─ E（东）：y 轴
    └─ D（地）：z 轴（正向向下）

PX4 里程计：NED
无人机机体系：FLU（前左上）
    ├─ F（前）：x 轴
    ├─ L（左）：y 轴
    └─ U（上）：z 轴
```

## 📊 状态机图

```
┌──────────┐
│  IDLE    │
└─────┬────┘
      │ 初始化完成
      ▼
┌──────────────┐
│  ARMING      │ ← 发送 ARM 命令、切换 OFFBOARD 模式
└─────┬────────┘
      │ 进入 OFFBOARD
      ▼
┌──────────────────┐
│  TRACKING        │ ← 主要工作状态（运行 DOB）
│  - PD 控制       │   - 定期检查是否需要降落
│  - DOB 补偿      │
└─────┬────────────┘
      │ 目标静止 > 5s
      │ 且正上方
      ▼
┌──────────────┐
│  DESCEND     │ ← 发送 LAND 命令
└──────────────┘
```

## 🔌 话题映射

### 订阅（Input）

| 话题 | 类型 | 频率 | 用途 |
|------|------|------|------|
| `/target_pose` | PoseStamped | 20 Hz | 目标位置（来自 ArUco） |
| `/fmu/out/vehicle_odometry` | VehicleOdometry | 30-50 Hz | 速度、姿态、位置 |
| `/fmu/out/vehicle_control_mode` | VehicleControlMode | 50 Hz | 检测 OFFBOARD 状态 |

### 发布（Output）

| 话题 | 类型 | 频率 | 内容 |
|------|------|------|------|
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | 30 Hz | ⭐ **主指令**：pos+vel+accel |
| `/fmu/in/offboard_control_mode` | OffboardControlMode | 30 Hz | 控制模式 flag |
| `/tracking/filtered_pose` | PoseStamped | 20 Hz | KF 输出位置 |
| `/tracking/estimated_velocity` | Twist | 30 Hz | 速度估计 |
| `/tracking/raw_pose` | PoseStamped | 20 Hz | 原始测量 |

## 🎯 性能指标

### 延迟

```
Measurement → Processing → Command
    ↓            ↓              ↓
  20 Hz      <5ms         30 Hz
   50ms
         35ms总延迟
```

### 频率

```
30 Hz 控制循环：33.3 ms
  │
  ├─ Step 1-6：~2 ms
  ├─ Step 7（发布）：~1 ms
  └─ 余量：~30 ms
```

## 🔧 配置参数体系

```
DroneTrackerController 参数
├─ drone 参数
│  ├─ mass (kg)
│  └─ hover_thrust_norm (0-1)
├─ control 参数
│  ├─ kp (0.5-2.0)
│  └─ kd (1.0-3.0)
├─ kf 参数
│  ├─ q_std (0.1-0.5)
│  ├─ r_std (0.001-0.05)
│  └─ dt (0.03 s)
├─ tracking 参数
│  └─ prediction_horizon (0.3-1.0 s)
└─ dob 参数
   ├─ K (1.0-5.0) [在代码中]
   └─ dt (0.03 s) [在代码中]
```

## 📈 数据流时序图

```
时间轴 ────────────────────────────────────────────→
     │
Camera (20Hz)
     │  ← 测量
     ├─ 20ms
     ├─ 20ms
     ├─ 20ms
     │
Control (30Hz)
     │  ← DOB 更新
     ├─ 10ms
     ├─ 10ms
     ├─ 10ms
     │
输出
     │  ← 指令发送
     ├─ 33ms
     ├─ 33ms
     ├─ 33ms

❌ 问题：Camera 和 Control 频率不同
✅ 解决：KF 内部处理不同时间步，DOB 自适应
```

---

**架构设计完成于**：2026-01-31

