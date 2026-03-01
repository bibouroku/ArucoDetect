# DOB 功能编译与测试指南

## 🔨 编译步骤

### 前置检查
```bash
# 确认 ROS2 环境
source /opt/ros/humble/setup.bash

# 确认工作区
cd ~/ros2_ws
ls -la src/tracktor-beam/
```

### 编译
```bash
# 清理旧构建（可选，但推荐）
rm -rf build install log

# 编译单个包
colcon build --packages-select tracktor_beam

# 如果需要详细日志
colcon build --packages-select tracktor_beam --event-handlers console_direct+

# 编译成功标志
# Finished `colcon build` [xx.xxs]
```

### 验证编译
```bash
# 检查安装
ls install/tracktor_beam/lib/
ls install/tracktor_beam/include/

# 重新加载环境
source install/setup.bash
```

---

## 🧪 测试场景

### 场景 A：静态测试（仅代码验证）

**目标**：验证 DOB 代码逻辑正确，无运行时错误

**步骤**：
```bash
# 1. 启动节点（会卡住等待测量）
ros2 launch tracktor_beam drone_tracker.launch.py &

# 2. 在另一终端发布测试消息
# (需要 PX4 或测试发布器)

# 3. 观察日志中是否有 "[DOB INFO]" 输出
# Ctrl+C 停止

# 预期结果：无段错误、无未定义符号、日志正常打印
```

### 场景 B：动态测试（实飞前仿真）

**目标**：在 Gazebo/SITL 环境中验证 DOB 与 PX4 交互

**准备**：
```bash
# 启动 PX4 SITL + Gazebo
cd ~/px4_autopilot
make px4_sitl gazebo

# 在另一终端启动 ROS2 bridge
ros2 launch micrortps_agent agent.launch.py
```

**运行**：
```bash
# 启动 drone_tracker
ros2 launch tracktor_beam drone_tracker.launch.py

# 监控关键话题
ros2 topic hz /fmu/out/vehicle_odometry      # 应约 30 Hz
ros2 topic hz /fmu/in/trajectory_setpoint    # 应约 30 Hz
```

**验证清单**：
- [ ] 无人机能正常起飞
- [ ] 收到目标位置信息
- [ ] 无人机跟踪目标
- [ ] 日志中出现 "[DOB INFO]"
- [ ] 日志中 Wind_Est 合理（无风时 < 0.2 N）

### 场景 C：风扰测试

**目标**：验证 DOB 抗风补偿效果

**准备**：
```bash
# 在 Gazebo 中添加风模型（可选）
# 或使用物理风扇（实飞环境）
```

**对比方法**：
```bash
# 方法1：临时禁用 DOB（为验证效果）
# 在代码中注释掉 DOB update 和补偿
# 重新编译，对比跟踪误差

# 方法2：监控日志差异
# 有风时 Wind_Est 应 > 0.5 N
# CmdAcc 应与 Wind_Est 相反（补偿）
```

**预期结果**：
- 启用 DOB：误差 0.2 m
- 禁用 DOB：误差 0.4+ m（有风时更明显）

---

## 📊 监控和调试

### 实时日志查看

**方法 1：过滤输出**
```bash
# 只看 DOB 相关日志
ros2 launch tracktor_beam drone_tracker.launch.py 2>&1 | grep "DOB"

# 看所有信息日志
ros2 launch tracktor_beam drone_tracker.launch.py 2>&1 | grep "INFO"

# 看错误和警告
ros2 launch tracktor_beam drone_tracker.launch.py 2>&1 | grep -E "ERROR|WARN"
```

**方法 2：ROS2 日志系统**
```bash
# 在另一终端设置日志级别
ros2 lifecycle set /drone_tracker_controller debug  # 或 info/warn/error

# 查看日志文件
tail -f ~/.ros/log/latest/*/drone_tracker_controller*
```

### 话题监控

```bash
# 查看估计的速度（应约 0.5 m/s）
ros2 topic echo /tracking/estimated_velocity

# 查看滤波后的位置
ros2 topic echo /tracking/filtered_pose

# 查看原始测量（未滤波）
ros2 topic echo /tracking/raw_pose
```

### 性能分析

```bash
# 测量话题延迟
ros2 topic delay /fmu/in/trajectory_setpoint

# 测量话题频率
ros2 topic hz /fmu/in/trajectory_setpoint
# 应输出 ~30 Hz
```

---

## ⚙️ 参数调试工作流

### 快速调整流程

**方法 1：命令行参数（推荐，无需重编）**
```bash
# 单个参数
ros2 launch tracktor_beam drone_tracker.launch.py kp:=1.2 kd:=2.5

# 多个参数
ros2 launch tracktor_beam drone_tracker.launch.py \
  kp:=1.2 kd:=2.5 mass:=2.5 q_std:=0.25

# 使用配置文件
ros2 launch tracktor_beam drone_tracker.launch.py \
  params_file:=$(pwd)/config_example_dob.yaml
```

**方法 2：参数文件编辑**
```bash
# 编辑参数文件
nano config_example_dob.yaml

# 保存后使用（需重启节点）
ros2 launch tracktor_beam drone_tracker.launch.py \
  params_file:=$(pwd)/config_example_dob.yaml
```

### 参数扫描测试

```bash
#!/bin/bash
# 测试不同的 kp 值

for kp in 0.8 1.0 1.2 1.5; do
    echo "Testing with kp=$kp"
    timeout 30 ros2 launch tracktor_beam drone_tracker.launch.py kp:=$kp
    sleep 5
done
```

---

## 📈 性能评估

### 定量指标测量

```bash
# 编写脚本测量跟踪误差
cat > measure_tracking_error.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np

class ErrorMeasurer(rclcpp.Node):
    def __init__(self):
        super().__init__('error_measurer')
        self.sub_raw = self.create_subscription(PoseStamped, '/tracking/raw_pose', 
                                                self.raw_callback, 10)
        self.sub_filt = self.create_subscription(PoseStamped, '/tracking/filtered_pose',
                                                 self.filt_callback, 10)
        self.raw_pos = None
        self.filt_pos = None
        
    def raw_callback(self, msg):
        self.raw_pos = [msg.pose.position.x, msg.pose.position.y]
        
    def filt_callback(self, msg):
        self.filt_pos = [msg.pose.position.x, msg.pose.position.y]
        
        if self.raw_pos and self.filt_pos:
            error = np.linalg.norm(np.array(self.raw_pos) - np.array(self.filt_pos))
            self.get_logger().info(f"Filtering error: {error:.4f} m")

if __name__ == '__main__':
    rclpy.init()
    node = ErrorMeasurer()
    rclpy.spin(node)
EOF

# 运行
python3 measure_tracking_error.py
```

### 定性观察

**优秀表现**：
- ✅ 平稳跟踪，无明显滞后
- ✅ 转向快速响应
- ✅ 短期误差 < 0.2m
- ✅ 日志中 Wind_Est 合理

**需要改进**：
- ⚠️ 跟踪落后 > 0.5m
- ⚠️ 响应有延迟或明显超调
- ⚠️ 日志显示 Wind_Est 异常大或不更新
- ⚠️ 有高频抖动

---

## 🐛 调试技巧

### 添加临时日志

在 `run_tracking_state()` 中添加：
```cpp
// 临时调试
RCLCPP_WARN(this->get_logger(),
    "[DEBUG] vel_measured=[%.3f, %.3f], dt=%.4f, accel=[%.3f, %.3f]",
    _vehicle_velocity_ned.x(), _vehicle_velocity_ned.y(),
    dt_control, current_accel.x(), current_accel.y());
```

编译后运行：
```bash
ros2 launch tracktor_beam drone_tracker.launch.py 2>&1 | grep DEBUG
```

### 隔离 DOB

```cpp
// 临时禁用 DOB（验证其效果）
// dob_->update(...);
// Eigen::Vector3d disturbance_accel = ...;
Eigen::Vector3d disturbance_accel = Eigen::Vector3d::Zero();  // 禁用补偿

// 比较性能差异
```

重新编译对比性能：
```bash
# 启用 DOB
ros2 launch tracktor_beam drone_tracker.launch.py > with_dob.log 2>&1

# 禁用 DOB（改代码后）
ros2 launch tracktor_beam drone_tracker.launch.py > without_dob.log 2>&1

# 对比日志
diff -u <(grep "PosErr" without_dob.log) <(grep "PosErr" with_dob.log)
```

### GDB 调试（如有段错误）

```bash
# 编译时添加调试符号（CMakeLists.txt）
# set(CMAKE_BUILD_TYPE Debug)

colcon build --packages-select tracktor_beam --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 用 GDB 运行
ros2 run tracktor_beam drone_tracker_controller __node:=drone_tracker_node &
gdb -p $(pidof drone_tracker_controller)

# GDB 命令
# (gdb) bt          # 显示调用栈
# (gdb) p 变量名    # 打印变量
# (gdb) c           # 继续执行
```

---

## 📋 测试清单

### 编译测试
- [ ] 代码编译无误
- [ ] 无 warning（至少没有新 warning）
- [ ] 可执行文件生成成功

### 启动测试
- [ ] 节点成功启动
- [ ] 订阅者和发布者正常运行
- [ ] 无段错误或运行时异常

### 功能测试
- [ ] 收到测量值后滤波输出正常
- [ ] 日志输出 "[DOB INFO]" 并有数值
- [ ] 发送给 PX4 的设定点包含加速度字段

### 性能测试
- [ ] 跟踪误差 < 0.3m（无风）
- [ ] 频率稳定 ~30Hz
- [ ] Wind_Est 在合理范围

### 参数调试测试
- [ ] 不同 kp 值影响响应速度
- [ ] 不同 K 值影响风补偿效果
- [ ] 参数改变实时生效（如支持）

---

## 🚀 部署前检查清单

```bash
# 最终验证脚本
cat > final_check.sh << 'EOF'
#!/bin/bash

echo "=== DOB 部署前检查清单 ==="

# 1. 编译检查
echo "[1/5] 检查编译..."
if colcon build --packages-select tracktor_beam > /dev/null 2>&1; then
    echo "✓ 编译成功"
else
    echo "✗ 编译失败"
    exit 1
fi

# 2. 文件检查
echo "[2/5] 检查必要文件..."
files=("src/drone_tracker/drone_tracker.hpp" \
       "src/drone_tracker/drone_tracker.cpp" \
       "DOB_IMPLEMENTATION_GUIDE.md")
for f in "${files[@]}"; do
    if [ -f "$f" ]; then
        echo "✓ $f"
    else
        echo "✗ 缺失 $f"
    fi
done

# 3. 代码检查
echo "[3/5] 检查关键代码..."
if grep -q "publish_full_trajectory_setpoint" src/drone_tracker/drone_tracker.cpp; then
    echo "✓ 完整轨迹发布函数存在"
else
    echo "✗ 缺失 publish_full_trajectory_setpoint"
fi

if grep -q "disturbance_accel" src/drone_tracker/drone_tracker.cpp; then
    echo "✓ DOB 补偿逻辑存在"
else
    echo "✗ 缺失 DOB 补偿逻辑"
fi

# 4. 参数检查
echo "[4/5] 检查参数..."
if grep -q "_vehicle_mass" src/drone_tracker/drone_tracker.hpp; then
    echo "✓ 质量参数定义"
else
    echo "✗ 缺失质量参数"
fi

# 5. 文档检查
echo "[5/5] 检查文档..."
docs=("DOB_IMPLEMENTATION_GUIDE.md" \
      "DOB_CHANGES_SUMMARY.md" \
      "DOB_QUICK_REFERENCE.md")
for d in "${docs[@]}"; do
    if [ -f "$d" ]; then
        echo "✓ $d"
    else
        echo "✗ 缺失 $d"
    fi
done

echo ""
echo "=== 检查完毕 ==="
EOF

chmod +x final_check.sh
./final_check.sh
```

---

## 🎓 常见编译错误解决

| 错误 | 原因 | 解决 |
|------|------|------|
| `undefined reference` | 头文件漏声明 | 检查 .hpp 中是否有函数声明 |
| `no member named` | 成员变量漏声明 | 检查 .hpp 中是否有成员变量定义 |
| `candidate constructor` | 函数参数类型不匹配 | 检查 update() 调用的三个参数类型 |
| `ISO C++` warning | 代码风格问题 | 通常可忽略，但推荐修复 |

---

**预计编译时间**：2-5 分钟  
**预计测试时间**：10-20 分钟  
**推荐测试顺序**：编译 → 启动 → 静态功能 → 动态仿真 → 参数调优

