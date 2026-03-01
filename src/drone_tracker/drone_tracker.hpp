#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <limits>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <stdint.h>
#include <iostream>

#define X_DIST 0.0
#define Y_DIST 0.0
#define HEIGHT -2.5

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "rclcpp/qos.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>
#include <Eigen/Geometry>

#include "kf_tracker/kf_tracker_lib.hpp"
#include <memory> // 为了 std::unique_ptr

#include <Eigen/Dense>

using namespace std::chrono_literals;


 //DOB controller functions
class DisturbanceObserver
{
public:
    // 构造函数
    // mass: 无人机质量 (kg)
    // K: 观测器增益矩阵的对角值 (对应论文中的 lambda_i [cite: 228])，通常取 2.0 - 5.0
    // dt: 控制周期 (s)
    DisturbanceObserver(double mass, double K, double dt)
        : mass_(mass), K_(K), dt_(dt)
    {
        fe_est_ = Eigen::Vector3d::Zero();

        beta_ = (dt_ * K_) / mass_;
        if (beta_ > 1.0) {
            beta_ = 1.0; // 限制 beta 不超过 1
        }
    }
    /**
     * @brief 更新观测器 (基于论文公式 15)
     * * @param accel_inertial   惯性系下的加速度 (m/s^2)。注意：这是测量值，通常来自 IMU 去除重力后的加速度 + 重力向量，或者直接微分速度。
     * 在 PX4 中，这通常是 (Velocity_new - Velocity_old) / dt。
     * @param R_body_to_earth  机体到惯性系(NED)的旋转矩阵 (对应论文中的 R(eta) 
     * @param thrust_force_n   总推力 (牛顿)。注意：PX4 发出的是 normalized thrust (0-1)，你需要乘一个系数转成牛顿。
     */
    void update(const Eigen::Vector3d& accel_inertial,
                const Eigen::Matrix3d& R_body_to_earth,
                double thrust_force_n)
    {
        // 1. 计算重力向量 (NED坐标系下重力是正 Z 方向)
        // 论文公式 q(4) 中 g = [0, 0, -mg0]^T (假设Z向上) [cite: 188]
        // 但在 PX4 NED 中，g = [0, 0, 9.8 * m]^T
        Eigen::Vector3d g_vec(0.0, 0.0, 9.81 * mass_);

        // 2. 计算推力向量在惯性系下的表示
        // 论文公式 (15) 中的 u_f * z_b 
        // z_b 是机体坐标系的 Z 轴在惯性系下的方向，即旋转矩阵的第三列 [cite: 228]
        // 在 PX4 NED 中，推力通常指向机体 -Z 方向 (向上)，所以推力产生的力是 R * [0, 0, -T]^T
        // 或者简单理解：推力向量 = R * [0, 0, -thrust_magnitude]
        Eigen::Vector3d thrust_body(0.0, 0.0, -thrust_force_n);
        Eigen::Vector3d thrust_inertial = R_body_to_earth * thrust_body;

        // 3. 计算“瞬时干扰力” (Raw Disturbance)
        // 动力学方程: F_total = m * a
        // F_total = F_gravity + F_thrust + F_disturbance
        // 所以: F_disturbance = m * a - F_gravity - F_thrust
        // 这对应论文公式 (15) 的括号部分: (m*p_dotdot - g + u_f*z_b)
        // 注意符号：论文里把 u_f*z_b 当作输入项，我们这里根据 NED 习惯做减法
        
        Eigen::Vector3d raw_disturbance = (mass_ * accel_inertial) - g_vec - thrust_inertial;

        // 4. 执行低通滤波 (论文公式 15 的离散化形式)
        // fe(k+1) = (1 - beta) * fe(k) + beta * raw_disturbance
        fe_est_ = (1.0 - beta_) * fe_est_ + beta_ * raw_disturbance;
    }

    // 获取估计的干扰加速度 (用于前馈补偿)
    // 论文估算的是力 f_e，我们要补偿的是加速度 a_comp = -f_e / m
    Eigen::Vector3d getDisturbanceAcceleration() const {
        return fe_est_ / mass_;
    }

    // 获取估计的干扰力 (牛顿)
    Eigen::Vector3d getDisturbanceForce() const {
        return fe_est_;
    }
private:
    double mass_;
    double K_;
    double dt_;
    double beta_;
    Eigen::Vector3d fe_est_;
};

class DroneTrackerController : public rclcpp::Node
{
public:
    void arm();
    void disarm();
    DroneTrackerController();
private:
    // 成员变量
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pj_raw_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pj_filtered_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pj_est_velocity_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::TimerBase::SharedPtr timer_takeoff_;
    VehicleControlMode c_mode;
    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
    //rclcpp::Time last_marker_time_;
    float base_x = 0.0;
    float base_y = 0.0;
    float base_z = 0.0f;           
    double aruco_x;
    double aruco_y;
    float aruco_z;
    float diff_x = 0.0;
    float diff_y = 0.0;
    float diff_z = 0.0;
    float reference_z_ = 0.0f;
    bool  reference_z_captured_ = false;
    bool  last_offboard_state_  = false;
    bool flag = true;
    bool is_directly_above_target() const;
    bool has_landed() const;
    rclcpp::Time _last_tag_seen_time;
    const float DESCEND_THRESHOLD_XY = 0.2f; // 水平距离小于这个值时开始下降 (米)
    const float LAND_THRESHOLD_Z = -0.1f; // 高度小于这个值时认为已降落 (米)

    std::unique_ptr<KFTrackerCore> kf_filter_;
    geometry_msgs::msg::PoseWithCovarianceStamped filtered_pose;

    struct ArucoTag {
		Eigen::Vector3d position;
		Eigen::Quaterniond orientation;
		rclcpp::Time timestamp;

		bool valid() { return timestamp.nanoseconds() > 0; };
	};
    ArucoTag _tag;
    ArucoTag _last_seen_tag; // 用于降落时锁定目标
    ArucoTag _last_stable_tag_position; //用于存储二维码上一个的稳定位置
    rclcpp::Time _last_tag_move_time; //记录二维码最后一次移动的时间
    bool _is_first_tag_detection {true};
    
    enum class State{
        IDLE,
        ARMING,
        TRACKING,
        DESCEND,
    };
    State current_state = State::IDLE;

    Eigen::Vector3d _vehicle_position_ned;
    Eigen::Quaterniond _vehicle_orientation;
    Eigen::Vector3d _vehicle_velocity_ned;              // 无人机速度
    rclcpp::Time _last_odometry_stamp;                 // 最新里程计时间戳
    Eigen::Vector3d _last_filtered_position;           // 上一次滤波位置（用于速度计算）
    rclcpp::Time _last_filter_time;                    // 上一次滤波时间
    double _prediction_horizon = 0.1;                  // 前向预测时间（秒）
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    //成员函数
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z);
    void publish_full_trajectory_setpoint(float x, float y, float z,
                                          float vx, float vy, float vz,
                                          float ax, float ay, float az);
    void publish_transform();
    void lookup_transform();
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0,float param2 = 0.0);
    ArucoTag getTagWorld(const ArucoTag& tag_camera);
    void switchToState(State state);
    std::string getStateName(State state);

    // ----- 状态机核心函数 -----
    void run_state_machine();
    void run_idle_state();
    void run_arming_state();
    void run_tracking_state();
    void run_descend_state();
    //void run_landed_state();

    std::unique_ptr<DisturbanceObserver> dob_; // X轴观测器
    std::unique_ptr<DisturbanceObserver> dob_y_; // Y轴观测器
    Eigen::Vector3d _last_cmd_accel; // 记录上一时刻的指令
    
    // --- DOB 相关成员变量 ---
    Eigen::Vector3d _last_vehicle_velocity;     // 上一帧的无人机速度（用于计算加速度）
    double _vehicle_mass = 2.0;                 // 无人机质量 (kg)，需要根据实际情况调整
    double _hover_thrust_norm = 0.5;            // 悬停时的标准化推力 (0-1)
    double _max_thrust_newton = 19.6;           // 最大推力 (牛顿)，= _vehicle_mass * 9.81 * 1.0
    
    // --- PD 控制器参数 ---
    double _kp = 1.0;                           // 位置增益
    double _kd = 2.0;                           // 速度增益
    
    // --- 控制指令缓存（用于反推推力）---
    double _current_normalized_thrust = 0.5;    // 当前标准化推力 (0-1)


    
    
    // ==================== 抖动修复成员变量 ====================
    // 加速度滤波
    Eigen::Vector3d _filtered_accel = Eigen::Vector3d::Zero();           // 滤波后的加速度
    double _accel_filter_alpha = 0.3;          // 低通滤波系数 (0.1-0.5)
    
    // 方法声明
    Eigen::Vector3d getFilteredAcceleration(const Eigen::Vector3d& raw_accel);
};

