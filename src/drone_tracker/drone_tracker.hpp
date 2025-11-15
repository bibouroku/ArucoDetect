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

using namespace std::chrono_literals;

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
};