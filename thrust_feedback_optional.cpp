#include "drone_tracker.hpp"
#include <px4_msgs/msg/actuator_motors.hpp>

/**
 * @file drone_tracker_thrust_feedback.cpp
 * @brief 可选：添加推力反馈订阅以改进 DOB 精度
 * 
 * 使用说明：
 * 1. 在 drone_tracker.hpp 中添加成员：
 *    rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_sub_;
 * 
 * 2. 在 DroneTrackerController 构造函数中调用此函数以订阅推力
 * 
 * 3. 删除或注释掉之前的固定 _current_normalized_thrust = 0.5 赋值
 */

void DroneTrackerController::subscribe_to_actuator_motors()
{
    auto actuator_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    
    actuator_motors_sub_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>(
        "/fmu/out/actuator_motors",
        actuator_qos,
        [this](const px4_msgs::msg::ActuatorMotors::SharedPtr msg) 
        {
            // PX4 的 ActuatorMotors 消息包含 8 个通道的控制信号 (通常前 4 个是电机)
            // control[0-3] 是四旋翼的四个电机，范围 [0, 1]
            
            if (msg->control.size() >= 4) {
                double avg_control = (
                    static_cast<double>(msg->control[0]) +
                    static_cast<double>(msg->control[1]) +
                    static_cast<double>(msg->control[2]) +
                    static_cast<double>(msg->control[3])
                ) / 4.0;
                
                _current_normalized_thrust = std::clamp(avg_control, 0.0, 1.0);
                
                // 可选：调试日志（降低频率避免刷屏）
                static int log_counter = 0;
                if (log_counter++ % 30 == 0) {  // 每 30 帧输出一次（30Hz 下约 1 秒）
                    RCLCPP_DEBUG(this->get_logger(),
                        "[THRUST] Motor: [%.3f, %.3f, %.3f, %.3f], Avg: %.3f",
                        msg->control[0], msg->control[1], 
                        msg->control[2], msg->control[3],
                        _current_normalized_thrust);
                }
            }
        }
    );
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to /fmu/out/actuator_motors for thrust feedback");
}

/**
 * 可选：替代方案 - 从里程计推力反馈（如果可用）
 * 某些 PX4 固件版本会在 VehicleOdometry 中提供推力估计
 */
void DroneTrackerController::extract_thrust_from_odometry(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    // 注意：不是所有 PX4 版本都在 VehicleOdometry 中提供推力信息
    // 这是一个示例实现，具体字段需根据你的 PX4 版本调整
    
    // 如果 VehicleOdometry 有推力信息（某些版本），可以这样提取：
    // _current_normalized_thrust = std::clamp(msg->thrust_normalized, 0.0, 1.0);
}

/**
 * 启动推力反馈订阅
 * 在 DroneTrackerController 构造函数中调用：
 * 
 *     subscribe_to_actuator_motors();
 */
