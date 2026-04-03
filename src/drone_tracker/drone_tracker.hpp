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
#include <px4_msgs/msg/vehicle_local_position.hpp>

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
#include <fstream>
#include <iomanip>
#include <deque>
#include <array>
#include <px4_msgs/msg/data_collect.hpp>

using namespace std::chrono_literals;


 //DOB controller functions
class DisturbanceObserver
{
public:
    // K_I_diag: 分轴增益 [Kx, Ky, Kz]（对应论文里 K_I 对角元），建议 2~8 起步
    DisturbanceObserver(double mass, const Eigen::Vector3d& K_I_diag, double dt)
        : mass_(mass), K_I_diag_(K_I_diag), dt_(dt)
    {
        fe_hat_ = Eigen::Vector3d::Zero();
        // 论文离散实现中出现 dt/(2m) * K_I
        alpha_ = (dt_ / (2.0 * mass_)) * K_I_diag_;
        // 限幅，避免数值发散（尤其 dt 抖或 K 过大）
        for (int i = 0; i < 3; ++i) {
            if (alpha_(i) < 0.0) alpha_(i) = 0.0;
            if (alpha_(i) > 1.0) alpha_(i) = 1.0;
        }
    }

    void reset(const Eigen::Vector3d& fe0 = Eigen::Vector3d::Zero()) {
        fe_hat_ = fe0;
    }

    // accel_ned: NED惯性系加速度 (m/s^2)，必须是 “p_ddot”
    // R_b2n: body -> NED 的旋转矩阵
    // u_f: 总推力幅值 (N)，正数
    void update(const Eigen::Vector3d& accel_ned,
                const Eigen::Matrix3d& R_b2n,
                double u_f)
    {
        // NED 下重力向量：+Z 方向（向下）为正
        const Eigen::Vector3d g_vec(0.0, 0.0, 9.81 * mass_);

        // z_b：机体系 z 轴在 NED 下方向 = R 的第3列
        const Eigen::Vector3d z_b = R_b2n.col(2);

        // 论文的“瞬时外力样本”：m*a - g + u_f*z_b
        // 在 NED 中：推力方向通常为 -z_b，所以 thrust_vector = -u_f*z_b
        // m*a = thrust_vector + g + f_e  => f_e = m*a - g - thrust_vector = m*a - g + u_f*z_b
        const Eigen::Vector3d fe_inst = (mass_ * accel_ned) - g_vec + (u_f * z_b);

        // 论文式(15)的一阶离散形式（分轴）：
        // fe_hat(k+1) = (I - alpha) * fe_hat(k) + alpha * fe_inst
        // 其中 alpha = dt/(2m) * K_I
        for (int i = 0; i < 3; ++i) {
            fe_hat_(i) = (1.0 - alpha_(i)) * fe_hat_(i) + alpha_(i) * fe_inst(i);
        }
    }

    Eigen::Vector3d getDisturbanceForce() const { return fe_hat_; }        // N
    Eigen::Vector3d getDisturbanceAcceleration() const { return fe_hat_ / mass_; } // m/s^2

private:
    double mass_;
    Eigen::Vector3d K_I_diag_;
    double dt_;
    Eigen::Vector3d alpha_;
    Eigen::Vector3d fe_hat_;
};

class LSWindEstimator
{
public:
    LSWindEstimator();
    double estimate_total_thrust(const std::array<float,4>& motor_speed, double battery) const;
    Eigen::Vector3d estimate_wind_force(const std::array<float,4>& motor_speed, double battery, const Eigen::Quaterniond& q, const Eigen::Vector3d& acceleration) const;

private:
    Eigen::Matrix<double, 1, 8> coefficients_;
    double intercept_{};
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
        HOLDING,
        TRACKING,
        DESCEND,
    };
    State current_state = State::IDLE;
    double hold_duration_sec_ = 4.5;
    bool hold_inited_ = false;
    rclcpp::Time hold_start_time_;
    Eigen::Vector3d hold_kp_ = Eigen::Vector3d(0.8, 0.8, 1.0); // HOLD 状态的 P 增益，Z 轴可以适当大一些
    Eigen::Vector3d hold_ki_ = Eigen::Vector3d(0.0, 0.0, 0.05); // HOLD 状态的 I 增益，初始为0，后续可调试开启
    Eigen::Vector3d hold_pos_ned_;
    Eigen::Vector3d hold_int_err_ = Eigen::Vector3d::Zero();  // 可选：PI   
    bool odom_received_ = false; // 标记是否已经收到过里程计数据
    int odom_count_ = 0; // 里程计消息计数器
    bool hold_wait_stable_ = false; // HOLD 状态是否等待位置稳定
    int hold_stable_count_ = 0; // HOLD 状态位置稳定计数器
    Eigen::Vector3d hold_last_pos_ned_{0.0, 0.0, 0.0}; // HOLD 状态上一次位置（用于判断稳定）

    bool offboard_and_arm_sent_ = false; // 记录是否已经发送过切换到 Offboard 模式和解锁的命令
    Eigen::Vector3d target_pos;

    Eigen::Vector3d _vehicle_position_ned;
    Eigen::Quaterniond _vehicle_orientation;
    Eigen::Vector3d _vehicle_velocity_ned;              // 无人机速度
    rclcpp::Time _last_odometry_stamp;                 // 最新里程计时间戳
    Eigen::Vector3d _last_filtered_position;           // 上一次滤波位置（用于速度计算）
    rclcpp::Time _last_filter_time;                    // 上一次滤波时间
    double _prediction_horizon = 0.1;                  // 前向预测时间（秒）

    float current_yaw_ = 0.0f; // 当前无人机航向角（弧度）
    float locked_yaw_ = 0.0f;

    void print_debug_panel();
    const char* state_to_string(State state) const;
    double wrap_pi(double angle) const;

    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

        
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_sub_;
    // 声明变量存储加速度
    Eigen::Vector3d _vehicle_accel_ned = Eigen::Vector3d::Zero();

    //成员函数
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z);
    void publish_full_trajectory_setpoint(float vx, float vy, float vz,
                                          float ax, float ay, float az,
                                          float yaw = std::numeric_limits<float>::quiet_NaN());
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
    void run_holding_state();
    void run_tracking_state();
    void run_descend_state();
    //void run_landed_state();

    std::unique_ptr<DisturbanceObserver> dob_; // X轴观测器
    std::unique_ptr<DisturbanceObserver> dob_y_; // Y轴观测器
    Eigen::Vector3d _last_cmd_accel; // 记录上一时刻的指令
    Eigen::Vector3d fe_hat;
    
    // --- DOB 相关成员变量 ---
    Eigen::Vector3d _last_vehicle_velocity;     // 上一帧的无人机速度（用于计算加速度）
    double _vehicle_mass = 2.0;                 // 无人机质量 (kg)，需要根据实际情况调整
    double _hover_thrust_norm = 0.5;            // 悬停时的标准化推力 (0-1)
    double _max_thrust_newton = 19.6;           // 最大推力 (牛顿)，= _vehicle_mass * 9.81 * 1.0

    //LS 风估计器
    rclcpp::Subscription<px4_msgs::msg::DataCollect>::SharedPtr data_collect_sub_;
    std::unique_ptr<LSWindEstimator> ls_wind_estimator_;
    std::deque<Eigen::Vector3d> wind_force_window_; // 用于平滑风力估计的窗口
    Eigen::Vector3d wind_force_est_{Eigen::Vector3d::Zero()}; // 当前风力估计

    std::array<float, 4> latest_rpm_{0.f, 0.f, 0.f, 0.f}; // 存储最新的电机转速
    double battery_voltage_ = 0.0;

    bool send_force_flag_{false};
    bool start_collect_{true};
    int average_size_{3};
    double wind_k_{1.5};

    Eigen::Vector3d thrust_world_test_{Eigen::Vector3d::Zero()}; // 用于调试的推力变量
    double f_test = 0.0;
    
    Eigen::Vector3d latest_collect_accel_{Eigen::Vector3d::Zero()}; // 存储最新的加速度数据
    Eigen::Quaterniond latest_collect_q_{Eigen::Quaterniond::Identity()};

    void data_collect_callback(const px4_msgs::msg::DataCollect::SharedPtr msg);

    // --- PD 控制器参数 ---
    double _kp = 1.0;                           // 位置增益
    double _kd = 2.0;                           // 速度增益
    // --- PID降落控制参数 ---
    double _descend_kp = 0.8;                  // 降落位置
    double _descend_kd = 1.0;                  // 降落速度
    double _descend_ki = 0.0;                  // 降落积分
    
    // --- 控制指令缓存（用于反推推力）---
    double _current_normalized_thrust = 0.5;    // 当前标准化推力 (0-1)
    

    
    
    // ==================== 抖动修复成员变量 ====================
    // 加速度滤波
    Eigen::Vector3d _filtered_accel = Eigen::Vector3d::Zero();           // 滤波后的加速度
    double _accel_filter_alpha = 0.3;          // 低通滤波系数 (0.1-0.5)
};

