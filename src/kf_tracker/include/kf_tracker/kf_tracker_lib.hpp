// #ifndef KF_TRACKER_NODE_H
// #define KF_TRACKER_NODE_H
// #include "rclcpp/rclcpp.hpp"
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// #include <stdio.h>
// #include <cstdlib>
// #include <string>
// #include <sstream>
// #include <functional>
// #include <Eigen/Dense>


// struct kf_state
// {
//     rclcpp::Time time_stamp;
//     Eigen::VectorXd x; // state vector
//     Eigen::MatrixXd P; // state covariance matrix   
// };

// struct sensor_measurement
// {
//     rclcpp::Time time_stamp;
//     Eigen::VectorXd z; // measurement vector
// };

// class KFTracker : public rclcpp::Node
// {
// public:
//     explicit KFTracker(const rclcpp::NodeOptions & options);

// private:
//     // ---- Kalman Filter Core Variables ----//
//     kf_state kf_state_pred_;
//     sensor_measurement z_meas_;
//     sensor_measurement z_last_meas_;
//     Eigen::MatrixXd F_;
//     Eigen::MatrixXd H_;
//     Eigen::MatrixXd Q_;
//     Eigen::MatrixXd R_;

//     double q_;
//     double r_;
//     double dt_pred_;
//     bool is_state_initialized_;
//     std::vector<kf_state> state_buffer_;
//     unsigned int state_buffer_size_;
//     bool do_update_step_;
//     double measurement_off_time_;
//     bool debug_;

//     // ---- Kalman Filter Core Methods ----//
//     void setQ(void);
//     void setR(void);
//     void setF(void);
//     void setH(void);
//     void initP(void);
//     void initKF(void);
//     void updateStateBuffer(void);
//     bool predict(void);
//     void update(void);

//     // ----- ROS2 Specific Methods And Members ---- //
//     void filterLoop(void);
//     void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
//     void publishState(void);
//     void declareAndGetParams();

//     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr state_pub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
//     rclcpp::TimerBase::SharedPtr kf_loop_timer_;

// };
// #endif

    
#ifndef KF_TRACKER_CORE_HPP
#define KF_TRACKER_CORE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Dense>
#include <vector>

// 前向声明，避免在头文件中包含完整的定义
struct kf_state
{
    rclcpp::Time time_stamp;
    Eigen::VectorXd x; // state vector
    Eigen::MatrixXd P; // state covariance matrix   
};

struct sensor_measurement
{
    rclcpp::Time time_stamp;
    Eigen::VectorXd z; // measurement vector
};

class KFTrackerCore
{
public:
    /**
     * @brief 构造函数，接收所有必要的参数
     * @param logger ROS 2 日志记录器，用于内部打印信息
     * @param q_std 过程噪声标准差
     * @param r_std 测量噪声标准差
     * @param dt_pred 预测步长时间
     */
    explicit KFTrackerCore(rclcpp::Logger logger, double q_std, double r_std, double dt_pred);

    /**
     * @brief 主处理函数：用一个新的测量值来更新滤波器状态
     * @param measurement 新的测量位姿（必须在世界坐标系下）
     */
    void updateWithMeasurement(const geometry_msgs::msg::PoseStamped::SharedPtr measurement);

    /**
     * @brief 获取当前滤波后的状态
     * @return 包含平滑后位姿和协方差的消息
     */
    geometry_msgs::msg::PoseWithCovarianceStamped getFilteredState() const;

    /**
     * @brief 检查滤波器是否已经初始化
     */
    bool isInitialized() const;
     Eigen::Vector3d getFilteredVelocity() const;

private:
    // 内部实现函数 (从你原来的代码中移动过来)
    void initKF();
    void setQ();
    void setR();
    void setF();
    void setH();
    void initP();
    bool predict();
    void update();
    void updateStateBuffer();

    // 成员变量 (从原来的代码中移动过来)
    rclcpp::Logger logger_; // 用于日志记录
    kf_state kf_state_pred_;
    sensor_measurement z_meas_;
    sensor_measurement z_last_meas_;
    Eigen::MatrixXd F_, H_, Q_, R_;
    double q_, r_, dt_pred_;
    bool is_state_initialized_;
    std::vector<kf_state> state_buffer_;
    unsigned int state_buffer_size_;
    bool debug_;

    // 注意：所有原来与ROS节点相关的成员 (Publisher, Subscriber, Timer) 都被移除了
};

#endif // KF_TRACKER_CORE_HPP

  