#include "drone_tracker.hpp"

static constexpr double STATIONARY_THRESHOLD_M = 0.05; // 5厘米，位置变化小于这个值被认为是静止
static constexpr double STATIONARY_DURATION_S = 5.0;   // 5秒，静止超过这个时间触发降落

using namespace std::chrono_literals; // 为了使用 30ms 这样的时间字面量

DroneTrackerController::DroneTrackerController() : Node("drone_tracker_controller"),current_state(State::IDLE) // 初始化节点名称
{
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    //初始化原始位姿发布方
    pj_raw_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tracking/raw_pose", 10);
    //初始化滤波后位姿发布方
    pj_est_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/tracking/estimated_velocity", 10);
    pj_filtered_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tracking/filtered_pose", 10);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",qos,std::bind(&DroneTrackerController::pose_callback, this, std::placeholders::_1));
    //tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //publish static transform from camera_link to aruco_marker
    //static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // Publish static transform from base_link to camera_links
    // geometry_msgs::msg::TransformStamped static_transform;
    // static_transform.header.stamp = this->now();
    // static_transform.header.frame_id = "base_link";
    // static_transform.child_frame_id  = "camera_link";
    // static_transform.transform.translation.x = 0.0;
    // static_transform.transform.translation.y = 0.0;
    // static_transform.transform.translation.z = 0.0;
    // static_transform.transform.rotation.x = 0.5;
    // static_transform.transform.rotation.y = -0.5; 
    // static_transform.transform.rotation.z = 0.5;
    // static_transform.transform.rotation.w = -0.5;
    // static_tf_broadcaster_->sendTransform(static_transform);

   vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&DroneTrackerController::odometry_callback, this, std::placeholders::_1));
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    vehicle_control_mode_subscriber_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos2,[this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg) { c_mode = *msg; });
    offboard_setpoint_counter_ = 0;
    // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // tf_buffer_->setUsingDedicatedThread(true); 

    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,this,false);
    reference_z_captured_ = false;
    last_offboard_state_  = false;

    double q_std = this->declare_parameter<double>("kf.q_std", 0.3);
    double r_std = this->declare_parameter<double>("kf.r_std", 0.01);
    double dt = this->declare_parameter<double>("kf.dt", 0.03);
    kf_filter_ = std::make_unique<KFTrackerCore>(this->get_logger(), q_std, r_std, dt);
    
    // 初始化速度和时间戳相关变量
    _vehicle_velocity_ned = Eigen::Vector3d::Zero();
    _last_tag_seen_time = this->now();
    _last_tag_move_time = this->now();

    _last_odometry_stamp = this->now();
    _last_filtered_position = Eigen::Vector3d::Zero();
    _last_filter_time = this->now();
    
    // 前向预测时间参数
    _prediction_horizon = this->declare_parameter<double>("tracking.prediction_horizon", 0.65);


    
    /*auto timer_callback = [this]() -> void {
        //publish_transform();
        //lookup_transform();
        // double intpart;
        // if (offboard_setpoint_counter_ == 10 && flag) {
        //     publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); //切换到OFFBOARD模式
        //     arm();
        //     flag = false;
        // }
        // if (modf(static_cast<double>(offboard_setpoint_counter_) / 2 , &intpart) == 0.0) {
        // publish_offboard_control_mode();
        // }
        // publish_trajectory_setpoint();

        ----------------状态机实现逻辑----------------

    };*/
    _last_tag_move_time = this->now();
    Eigen::Vector3d ki_diag = Eigen::Vector3d(2.0, 2.0, 0.0); // 只对X和Y轴进行积分补偿，Z轴不补偿
    dob_ = std::make_unique<DisturbanceObserver>(2.0, ki_diag, 0.03);  // mass=2.0kg, K=2.0, dt=0.03s (30ms)
    dob_y_ = std::make_unique<DisturbanceObserver>(2.0, ki_diag, 0.03); // Y轴观测器
    _last_cmd_accel = Eigen::Vector3d::Zero();
    _last_vehicle_velocity = Eigen::Vector3d::Zero();
    _filtered_accel = Eigen::Vector3d::Zero();  // ✅ 新增：初始化滤波加速度
    _vehicle_mass = this->declare_parameter<double>("drone.mass", 2.0);
    _hover_thrust_norm = this->declare_parameter<double>("drone.hover_thrust_norm", 0.5);
    //_max_thrust_newton = _vehicle_mass * 9.81;
    // 用 hover_thrust_norm 反推 max_thrust，使 hover_thrust_norm 对应 mg
    // max_thrust ≈ (m*g) / hover_thrust_norm
    _max_thrust_newton = (_vehicle_mass * 9.81) / std::max(0.05, _hover_thrust_norm);
    _kp = this->declare_parameter<double>("control.kp", 1.0);
    _kd = this->declare_parameter<double>("control.kd", 2.0);
    
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vehicle_local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", 
        sensor_qos, 
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            // 直接读取 NED 系下的加速度
            // 注意：PX4 的 VLP 消息中 ax, ay, az 定义为 "Acceleration in NED frame"
            // 且通常已经去除了重力
            _vehicle_accel_ned = Eigen::Vector3d(msg->ax, msg->ay, msg->az);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "Received VehicleLocalPosition !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            // 如果你仍然觉得有噪声，可以在这里加个轻微的低通滤波，但通常不需要
            // _vehicle_accel_ned = getFilteredAcceleration(_vehicle_accel_ned);
        });
    timer_ = this->create_wall_timer(30ms, std::bind(&DroneTrackerController::run_state_machine, this));
}

void DroneTrackerController::run_state_machine()
{
    // 持续发布 OffboardControlMode 信号，这是PX4的要求
    publish_offboard_control_mode();

    // 根据当前状态调用对应的处理函数
    switch (current_state) {
        case State::IDLE:
            run_idle_state();
            break;
        case State::ARMING:
            run_arming_state();
            break;
        case State::HOLDING:
            run_holding_state();
            break;
        case State::TRACKING:
            run_tracking_state();
            break;
        case State::DESCEND:
            run_descend_state();
            break;
        // case State::FINISHED:
        //     run_landed_state();
        //     break;
    }
}


void DroneTrackerController::run_idle_state()
{
    RCLCPP_INFO(this->get_logger(), "State: IDLE");
    switchToState(State::ARMING);
}

void DroneTrackerController::run_arming_state()
{
    // 发送命令进入Offboard模式并解锁
    // PX4需要先接收到一段时间的setpoint流才能切换到offboard模式
    publish_offboard_control_mode();
    publish_trajectory_setpoint(0,0,HEIGHT); // 发布一个初始设定点，帮助 PX4 切换到 Offboard 模式
    if (!offboard_and_arm_sent_ && offboard_setpoint_counter_ > 10 ) {
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // 切换到OFFBOARD模式
        arm();
        offboard_and_arm_sent_ = true; // 记录已经发送过命令，避免重复发送
    }
    // 检查是否已经切换到Offboard模式
    if (c_mode.flag_control_offboard_enabled) {
        switchToState(State::HOLDING);
    }
    offboard_setpoint_counter_++;
}

void DroneTrackerController::run_holding_state()
{
    publish_offboard_control_mode();
    RCLCPP_INFO(this->get_logger(), "State: HOLDING");
    if (!hold_inited_) {
        hold_start_time_ = this->now();
        hold_inited_ = true;

        hold_pos_ned_ = _vehicle_position_ned;
        hold_pos_ned_.z() = HEIGHT;   // 你的 HEIGHT 是固定高度（NED）
        hold_int_err_.setZero();
    }
    const double dt = 0.03; // 你的 timer 周期；如果你有更真实dt可替换

    // 位置误差
    Eigen::Vector3d e = hold_pos_ned_ - _vehicle_position_ned;

    // 可选 PI：积分抗风漂
    hold_int_err_ += e * dt;
    // 防 windup
    hold_int_err_.x() = std::clamp(hold_int_err_.x(), -2.0, 2.0);
    hold_int_err_.y() = std::clamp(hold_int_err_.y(), -2.0, 2.0);
    hold_int_err_.z() = std::clamp(hold_int_err_.z(), -2.0, 2.0);

    Eigen::Vector3d v_sp = hold_kp_ * e + hold_ki_ * hold_int_err_;

    // 限幅（先保守）
    v_sp.x() = std::clamp(v_sp.x(), -2.0, 2.0);
    v_sp.y() = std::clamp(v_sp.y(), -2.0, 2.0);
    v_sp.z() = std::clamp(v_sp.z(), -1.0, 1.0);

    // DOB 前馈（只做补偿，不做闭环）
    Eigen::Vector3d a_ff = Eigen::Vector3d::Zero();
    // 如果你已经订阅了 vehicle_local_position 的加速度，建议直接用它更新DOB
    // dob_->update(_vehicle_accel_ned, R_body_to_earth, thrust_newton);
    // a_ff = - dob_->getDisturbanceAcceleration();   // 注意符号：补偿用负号

    publish_full_trajectory_setpoint(v_sp.x(), v_sp.y(), v_sp.z(),
                               a_ff.x(), a_ff.y(), a_ff.z());
    const double t = (this->now() - hold_start_time_).seconds();
    if (t >= hold_duration_sec_) {
        switchToState(State::TRACKING);
    }
}

void DroneTrackerController::run_tracking_state()
{
    publish_offboard_control_mode();
    // 发布目标位置作为设定点
    // publish_trajectory_setpoint(_tag.position.x(), _tag.position.y(), HEIGHT);
    // RCLCPP_INFO(this->get_logger(), "TRACKING: Following tag at [%.2f, %.2f, %.2f]",
    //             _tag.position.x(), _tag.position.y(), HEIGHT); 
    // ✅ 关键修复：使用卡尔曼滤波估计的速度进行前向预测
    const double tag_timeout_s = 2.0;
    const bool tag_recent = ((this->now() - _last_tag_seen_time).seconds() <= tag_timeout_s);
    if (!kf_filter_->isInitialized()) {
        // 滤波器尚未初始化：先保持当前速度为 0，同时用 vz 纠正高度
        const double z_err = HEIGHT - _vehicle_position_ned.z();
        const double vz_cmd = std::clamp(_kp * z_err, -1.0, 1.0);
        publish_full_trajectory_setpoint(0.0f, 0.0f, static_cast<float>(vz_cmd), 0.0f, 0.0f, 0.0f);
        return;
    }
    // 检查是否长时间没有看到二维码
    if (!tag_recent) {
        // 丢失目标：悬停在最后一次看到的目标位置（仍然用速度闭环，不发送位置）
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Tag lost for %.2f s, holding last seen target with velocity control.",
                             (this->now() - _last_tag_seen_time).seconds());
        target_pos = _last_seen_tag.position;
    }else{
        auto filtered_state = kf_filter_->getFilteredState();
        target_pos = Eigen::Vector3d(filtered_state.pose.pose.position.x,
                                        filtered_state.pose.pose.position.y,
                                        filtered_state.pose.pose.position.z);
    }
    target_pos.z() = HEIGHT;  // 强制高度锁定到固定高度（NED）
    // Eigen::Vector3d current_filtered_position(
    //     filtered_state.pose.pose.position.x,
    //     filtered_state.pose.pose.position.y,
    //     filtered_state.pose.pose.position.z
    // );
    
    // ✅ 关键修复：第一次运行时，初始化 _last_filtered_position 并返回
    // rclcpp::Time current_time = this->now();
    // double dt_since_last = (current_time - _last_filter_time).seconds();
    
    // if (dt_since_last < 0.01)  // 第一次调用或间隔太短，初始化状态
    // {
    //     _last_filtered_position = current_filtered_position;
    //     _last_filter_time = current_time;
    //     return;
    // }

    // Eigen::Vector3d predicted_position = current_filtered_position;
    // Eigen::Vector3d velocity_estimate = Eigen::Vector3d::Zero();
    // Eigen::Vector3d kf_velocity = kf_filter_->getFilteredVelocity();



    // ✅ 先计算加速度大小（用于后续自适应）
    // 需要在使用前定义
    //double dt_control = 0.03; // 控制周期 (30ms)
    //Eigen::Vector3d current_accel = _vehicle_accel_ned;

    // ✅ 改进：使用低通滤波后的加速度，减少噪声导致的 DOB 过度响应
    //Eigen::Vector3d current_accel = getFilteredAcceleration(current_accel_raw);
    
    // 计算加速度大小（用于后续自适应）
   // double accel_magnitude = std::sqrt(current_accel.x() * current_accel.x() + 
                                       //current_accel.y() * current_accel.y());

    /*计算滤波器估计的速度（基于位置变化）
    if (dt_since_last > 0.02 && dt_since_last < 0.5)
    {
        velocity_estimate = (current_filtered_position - _last_filtered_position) / dt_since_last;
        // 清零 Z 方向速度（小车在地面，Z不动）
        velocity_estimate.z() = 0.0;

        // ✅ 改进：根据加速度大小自动调整预测地平线
        // 原理：小车加速时，用旧速度预测会过度前推，导致目标位置突跳
        // 解决：加速度大时，缩小预测地平线
        double adaptive_horizon = _prediction_horizon;
        if (accel_magnitude > 0.5)  // 检测到加速阶段（加速度 > 0.5 m/s²）
        {
            adaptive_horizon = _prediction_horizon * 0.5;  // 减半预测地平线
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50,
                "[ADAPTIVE] Accel detected (%.3f m/s²): reducing horizon from %.3f to %.3f s",
                accel_magnitude, _prediction_horizon, adaptive_horizon);
        }
        
        // 使用自适应地平线进行预测
        predicted_position = current_filtered_position; //ss+ velocity_estimate * adaptive_horizon;
        
        double speed_xy = std::sqrt(velocity_estimate.x() * velocity_estimate.x() + 
                                    velocity_estimate.y() * velocity_estimate.y());
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                    "[VELOCITY] kf_vx=%.4f, kf_vy=%.4f, speed_xy=%.4f m/s, dt=%.3f s, horizon=%.3f s, predicted_delta=[%.4f, %.4f]",
                    kf_velocity.x(), kf_velocity.y(), speed_xy, dt_since_last,
                    _prediction_horizon,
                    kf_velocity.x() * _prediction_horizon,
                    kf_velocity.y() * _prediction_horizon);

        // ✅ 发布估计的速度（用于 PlotJuggler）
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = velocity_estimate.x();
        vel_msg.linear.y = velocity_estimate.y();
        vel_msg.linear.z = 0.0;
        pj_est_velocity_pub_->publish(vel_msg);
    }
    // ✅ 重要：更新上一次滤波位置和时间（必须在速度计算和使用之后）
    _last_filtered_position = current_filtered_position;
    _last_filter_time = current_time;*/
    
    // 发布预测位置而不是当前位置
    //publish_trajectory_setpoint(predicted_position.x(), predicted_position.y(), HEIGHT);

    // 1. 获取目标状态 (来自卡尔曼滤波)
    //Eigen::Vector3d target_pos = predicted_position;
  //  Eigen::Vector3d target_vel = kf_filter_->getFilteredVelocity();
    // 假设目标加速度为0 (或者如果是轨迹跟踪，这里应该有目标加速度)
 //   Eigen::Vector3d target_acc = Eigen::Vector3d::Zero(); 

    // 2. 基础 PID 控制 (计算标称控制量 u_nominal)
    // 这里的参数需要根据你的无人机响应调整，或者直接让 PX4 处理位置环，我们只做前馈
    // 但为了 DOB 生效，我们最好自己计算一个期望加速度作为基准
  //  Eigen::Vector3d pos_error = target_pos - _vehicle_position_ned;
 //   Eigen::Vector3d vel_error = target_vel - _vehicle_velocity_ned;
    
    // 简单的 PD 控制器生成期望加速度
   // Eigen::Vector3d cmd_accel_nominal = _kp * pos_error + _kd * vel_error + target_acc;

    // ✅ 更新速度（用于下次加速度计算）
  //  _last_vehicle_velocity = _vehicle_velocity_ned;


   //-----------------------------V2.0--------------------------------//
    // 2) 目标速度前馈（来自 KF 或者位置差分估计）
    Eigen::Vector3d target_vel_ff = kf_filter_->getFilteredVelocity();
    target_vel_ff.z() = 0.0; // 只做平面速度前馈
    rclcpp::Time current_time = this->now();
    double dt_since_last = (current_time - _last_filter_time).seconds();
    if (dt_since_last > 0.02 && dt_since_last < 0.5) {
        Eigen::Vector3d vel_diff = (target_pos - _last_filtered_position) / dt_since_last;
        vel_diff.z() = 0.0;

        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = vel_diff.x();
        vel_msg.linear.y = vel_diff.y();
        vel_msg.linear.z = 0.0;
        pj_est_velocity_pub_->publish(vel_msg);
    }
    _last_filtered_position = target_pos;
    _last_filter_time = current_time;

    // 3) 核心改动：由位置误差 -> 期望速度，不发送目标位置
    Eigen::Vector3d pos_error = target_pos - _vehicle_position_ned;
    pos_error.z() = HEIGHT - _vehicle_position_ned.z(); // 再次确保高度误差正确

    // 速度误差（用来加一点阻尼/跟随前馈速度）
    Eigen::Vector3d vel_error = target_vel_ff - _vehicle_velocity_ned;
    vel_error.z() = 0.0 - _vehicle_velocity_ned.z();

    // v_cmd = Kp * (p_sp - p) + Kd * (v_ff - v)
    Eigen::Vector3d v_cmd = _kp * pos_error + _kd * vel_error + target_vel_ff;  //加上速度前馈

    // 速度限幅（按需调参）
    const double vxy_max = 2.5;
    const double vz_max  = 1.0;
    v_cmd.x() = std::clamp(v_cmd.x(), -vxy_max, vxy_max);
    v_cmd.y() = std::clamp(v_cmd.y(), -vxy_max, vxy_max);
    v_cmd.z() = std::clamp(v_cmd.z(), -vz_max,  vz_max);

    // 4) DOB：估计外扰并作为加速度前馈补偿（只补偿 XY）
    Eigen::Vector3d current_accel = _vehicle_accel_ned;
    // 2. 获取当前旋转矩阵 R (将四元数转为 Eigen::Matrix3d)
    Eigen::Matrix3d R_body_to_earth = _vehicle_orientation.toRotationMatrix();

    // 2) 推力幅值 u_f（如果没有真实反馈，先用 hover 近似 + 简单修正）
    // 最保守：直接用悬停推力（适合你“只想估风力”且机动不大）：
    double u_f = _hover_thrust_norm * _max_thrust_newton;

    // 3. 获取当前总推力 (单位：牛顿)
    // 推力 = 标准化推力 × 最大推力
    // 注意：这里 _current_normalized_thrust 是从 PX4 的油门指令反推的
    // 简化方案：假设我们发送的加速度指令会被 PX4 转换为相应的推力
    // 更准确的做法是从 PX4 的实际推力反馈获取，但这里先使用估计值
    //double thrust_newton = _current_normalized_thrust * _max_thrust_newton;

    // --- 正确调用 DOB 更新 (对应论文公式 15) ---
    // DOB 观测器需要：当前加速度、旋转矩阵、推力
    dob_->update(current_accel, R_body_to_earth, u_f);
    Eigen::Vector3d fe_hat = dob_->getDisturbanceForce();   // N
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50,
        "[DOB] fe_hat[N] = [%.2f, %.2f, %.2f], u_f=%.2f N, accel=[%.2f, %.2f, %.2f]",
        fe_hat.x(), fe_hat.y(), fe_hat.z(),
        u_f,
        current_accel.x(), current_accel.y(), current_accel.z());
    // 获取估计的干扰力，换算成补偿加速度
    Eigen::Vector3d disturbance_force = dob_->getDisturbanceForce();
    Eigen::Vector3d disturbance_accel = disturbance_force / _vehicle_mass;


    // 注意符号：补偿时取负号；只补偿 XY，Z=0
    Eigen::Vector3d a_ff(-disturbance_accel.x(), -disturbance_accel.y(), 0.0);
    // 5) 发送速度 + 加速度（position 全 NaN，由 publish_full_trajectory_setpoint 保证）
    publish_full_trajectory_setpoint(static_cast<float>(v_cmd.x()),
                                     static_cast<float>(v_cmd.y()),
                                     static_cast<float>(v_cmd.z()),
                                     static_cast<float>(a_ff.x()),
                                     static_cast<float>(a_ff.y()),
                                     static_cast<float>(a_ff.z()));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                         "TRACKING: target_xy=[%.3f, %.3f], v_cmd=[%.3f, %.3f, %.3f], a_ff=[%.3f, %.3f]",
                         target_pos.x(), target_pos.y(),
                         v_cmd.x(), v_cmd.y(), v_cmd.z(),
                         a_ff.x(), a_ff.y());
    // ✅ 改进：在加速阶段减弱 DOB 补偿
    // 原理：小车加速产生的加速度是真实物理事件，不是风扰
    // 过度补偿会导致超调。所以加速度大时，减弱补偿
    //Eigen::Vector3d disturbance = disturbance_accel;
    
    // if (accel_magnitude > 0.5)
    // {
    //     // 在加速阶段，只补偿一半的估计干扰
    //     disturbance *= 0.5;
        
    //     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    //         "[ADAPTIVE DOB] Reducing compensation by 50percent during acceleration (%.3f m/s²)",
    //         accel_magnitude);
    // }
    
    // 4. 计算最终指令 (补偿干扰)
    // u_final = u_nominal - d_hat (减弱的干扰)
    //Eigen::Vector3d cmd_accel_final = cmd_accel_nominal - disturbance;
    
    // 只补偿 XY 平面，Z 轴保持原样或由 PX4 内部处理
    //cmd_accel_final.z() = 0.0; 

    // 记录指令供下一次迭代使用
    //_last_cmd_accel = cmd_accel_final;

    // 5. 发送带有加速度前馈的设定点
    // 我们依然发送位置 setpoint，但把计算出的加速度填入 acceleration 字段
    // PX4 会把这个 acceleration 直接加到它的速度环输出上


    /*publish_full_trajectory_setpoint(
        target_pos.x(), target_pos.y(), HEIGHT,    // 位置
    //    target_vel.x(), target_vel.y(), 0.0,        // 速度前馈
        disturbance_accel.x(), disturbance_accel.y(), 0.0 // 加速度前馈 (包含 DOB 补偿)
    );*/


    //publish_trajectory_setpoint(target_pos.x(), target_pos.y(), HEIGHT);
    // Log 调试信息
    // RCLCPP_INFO(this->get_logger(), 
    //             "[DOB INFO] Wind_Est: [%.3f, %.3f] N, Accel_Est: [%.3f, %.3f] m/s², "
    //             "Accel_Mag: %.3f, CmdAcc: [%.3f, %.3f] m/s², PosErr: [%.3f, %.3f] m",
    //             disturbance_force.x(), disturbance_force.y(),
    //             disturbance_accel.x(), disturbance_accel.y(),
    //             accel_magnitude,  // ✅ 新增
    //             cmd_accel_final.x(), cmd_accel_final.y(),
    //             pos_error.x(), pos_error.y());

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
    //             "TRACKING: Following tag at [%.3f, %.3f, %.2f] (predicted)",
    //             predicted_position.x(), predicted_position.y(), HEIGHT);

    // 检查是否满足进入降落状态的条件
    if ((this->now() - _last_tag_move_time).seconds() > STATIONARY_DURATION_S &&
        is_directly_above_target()) {
        RCLCPP_INFO(this->get_logger(), "Tag has been stationary for %.1f seconds. Initiating DESCEND state.",
                    STATIONARY_DURATION_S);
        switchToState(State::DESCEND);
    }
}

// 检查是否在目标正上方
bool DroneTrackerController::is_directly_above_target() const
{
    double dx = _vehicle_position_ned.x() - _tag.position.x();
    double dy = _vehicle_position_ned.y() - _tag.position.y();
    return std::sqrt(dx * dx + dy * dy) < DESCEND_THRESHOLD_XY;
}

// 检查是否已经降落
bool DroneTrackerController::has_landed() const
{
    // 一个简单的实现：检查高度是否非常接近地面
    // 更好的实现会检查z速度是否也接近0
    return _vehicle_position_ned.z() > LAND_THRESHOLD_Z;
}

void DroneTrackerController::run_descend_state()
{
    publish_offboard_control_mode();
    // // 发布锁定的降落位置作为设定点，逐渐降低高度
    // float descend_z = std::max(_last_seen_tag.position.z() - 0.1f, 0.0f); // 每次降低0.1米，最低到地面
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    // publish_trajectory_setpoint(_last_seen_tag.position.x(), _last_seen_tag.position.y(), descend_z);
    // RCLCPP_INFO(this->get_logger(), "DESCEND: Descending to [%.2f, %.2f, %.2f]",
    //             _last_seen_tag.position.x(), _last_seen_tag.position.y(), );

    // 检查是否已经降落
    if (has_landed()) {
        RCLCPP_INFO(this->get_logger(), "Landed successfully.");
        //switchToState(State::FINISHED);
    }
}

void DroneTrackerController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    // geometry_msgs::msg::TransformStamped transform;
    // transform.header.stamp = this->now();
    // transform.header.frame_id = "camera_link";
    // transform.child_frame_id  = "aruco_marker";
    // transform.transform.translation.x = msg->pose.position.x;
    // transform.transform.translation.y = msg->pose.position.y;
    // transform.transform.translation.z = msg->pose.position.z;
    // transform.transform.rotation.x = msg->pose.orientation.x;
    // transform.transform.rotation.y = msg->pose.orientation.y;
    // transform.transform.rotation.z = msg->pose.orientation.z;
    // transform.transform.rotation.w = msg->pose.orientation.w;
    // tf_broadcaster_->sendTransform(transform);

    // auto tag_camera = ArucoTag{
    //     .position = Eigen::Vector3d(
    //         msg->pose.position.x,
    //         msg->pose.position.y,
    //         msg->pose.position.z),
    //     .orientation = Eigen::Quaterniond(msg->pose.orientation.w,
    //                                      msg->pose.orientation.x,
    //                                      msg->pose.orientation.y,
    //                                      msg->pose.orientation.z),
    //     .timestamp = this->now()
    // };
    ArucoTag tag_camera;
    tag_camera.position = Eigen::Vector3d(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z);
    tag_camera.orientation = Eigen::Quaterniond(msg->pose.orientation.w,
                                               msg->pose.orientation.x,
                                               msg->pose.orientation.y,
                                               msg->pose.orientation.z);
    tag_camera.timestamp = this->get_clock()->now();
    _tag = getTagWorld(tag_camera);

    geometry_msgs::msg::PoseStamped::SharedPtr initial_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    initial_msg->header.stamp = _tag.timestamp;
    initial_msg->pose.position.x = _tag.position.x();
    initial_msg->pose.position.y = _tag.position.y();
    initial_msg->pose.position.z = _tag.position.z();
    initial_msg->pose.orientation.x = _tag.orientation.x();
    initial_msg->pose.orientation.y = _tag.orientation.y();
    initial_msg->pose.orientation.z = _tag.orientation.z();
    initial_msg->pose.orientation.w = _tag.orientation.w();
    // 发布原始位姿
    geometry_msgs::msg::PoseStamped raw_msg;
    raw_msg.header = initial_msg->header;
    raw_msg.pose = initial_msg->pose;
    pj_raw_pose_pub_->publish(raw_msg);
    // // ✅ 添加延迟诊断信息
    // rclcpp::Time current_time = this->now();
    // double measurement_delay = (current_time - msg->header.stamp).seconds();
    // if (measurement_delay > 0.01)  // 延迟 > 10ms 时输出
    // {
    //     RCLCPP_WARN(this->get_logger(), 
    //                 "[DELAY DETECTED] Measurement delay: %.1f ms | Meas time: %.3f | Current time: %.3f",
    //                 measurement_delay * 1000.0, msg->header.stamp.seconds(), current_time.seconds());
    // }
    
    // 使用卡尔曼滤波器进行位置和速度估计
    kf_filter_->updateWithMeasurement(initial_msg);
    if (!kf_filter_->isInitialized()) {
        return;
    }

    // 2. 从滤波器获取平滑后的位姿
    filtered_pose = kf_filter_->getFilteredState();

    // 发布滤波后位姿
    geometry_msgs::msg::PoseStamped filtered_msg;
    filtered_msg.header = filtered_pose.header;
    filtered_msg.pose = filtered_pose.pose.pose;;
    pj_filtered_pose_pub_->publish(filtered_msg);
    
    _last_tag_seen_time = this->now();
    _last_seen_tag = _tag; 
    if (_is_first_tag_detection)
    {
        _last_stable_tag_position = _tag;
        _last_tag_move_time = this->now();
        _is_first_tag_detection = false;
    }
    else
    {
        double distance_moved = (_tag.position - _last_stable_tag_position.position).norm();
        if (distance_moved > STATIONARY_THRESHOLD_M)
        {
            _last_tag_move_time = this->now();
            _last_stable_tag_position = _tag;
        }
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
        "Transform from ned to april tag(filtered):");
    // RCLCPP_INFO(this->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f",
    //             _tag.position.x(),
    //             _tag.position.y(),
    //             _tag.position.z());
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
        "Translation: x=%.2f, y=%.2f, z=%.2f",
        filtered_pose.pose.pose.position.x,
        filtered_pose.pose.pose.position.y,
        filtered_pose.pose.pose.position.z);

}
DroneTrackerController::ArucoTag DroneTrackerController::getTagWorld(const ArucoTag& tag_camera) {
    Eigen::Matrix3d R;
    R << 0, -1, 0,
        1, 0, 0,
        0, 0, 1;
    Eigen::Quaterniond quat_NED(R);

    // 计算测量时间和当前时间的差值
    // rclcpp::Time meas_time = tag_camera.timestamp;
    // rclcpp::Time current_time = this->now();
    // double time_diff = (current_time - meas_time).seconds();

    auto vehicle_position = Eigen::Vector3d(_vehicle_position_ned.cast<double>());
    auto vehicle_orientation = Eigen::Quaterniond(_vehicle_orientation.cast<double>());
    Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
    Eigen::Affine3d camera_transform = Eigen::Translation3d(0,0,0) * quat_NED;
    Eigen::Affine3d tag_transform = Eigen::Translation3d(tag_camera.position) * tag_camera.orientation;
    Eigen::Affine3d tag_transform_world = drone_transform * camera_transform * tag_transform;

    // ArucoTag tag_world = {
    //     .position = tag_transform_world.translation(),
    //     .orientation = Eigen::Quaterniond(tag_transform_world.rotation()),
    //     .timestamp = tag_camera.timestamp
    // };
    ArucoTag tag_world;
    tag_world.position = tag_transform_world.translation();
    tag_world.orientation = Eigen::Quaterniond(tag_transform_world.rotation());
    tag_world.timestamp = tag_camera.timestamp;
    
    // 调试输出
    // if(time_diff > 0.01)  // 如果延迟 > 10ms 才输出
    // {
    //     RCLCPP_WARN(this->get_logger(), 
    //                 "Measurement delay detected: %.3f ms", time_diff * 1000.0);
    // }

    return tag_world;
}

void DroneTrackerController::arm(){
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void DroneTrackerController::disarm(){
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void DroneTrackerController::publish_transform(){
    // ros2 run tf2_ros static_transform_publisher 
    // --x 0 --y 0 --z 0 --qx 0.707 --qy -0.707 --qz 0 --qw 0 --frame-id base_link --child-frame-id camera_link
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id  = "ned";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 1.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 0.0;
    tf_broadcaster_->sendTransform(transform);
}
void DroneTrackerController::lookup_transform(){
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string target_frame = "ned";
    std::string source_frame = "aruco_marker";
    try{
        transformStamped = tf_buffer_->lookupTransform(target_frame, source_frame,
                             tf2::TimePointZero);
        RCLCPP_INFO(this->get_logger(), "Transform from map to april tag:");
        RCLCPP_INFO(this->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f",
                    transformStamped.transform.translation.x,
                    transformStamped.transform.translation.y,
                    transformStamped.transform.translation.z);
        aruco_x = transformStamped.transform.translation.x;
        aruco_y = transformStamped.transform.translation.y;
        aruco_z = transformStamped.transform.translation.z;
        diff_x = aruco_x - X_DIST;
        diff_y = aruco_y - Y_DIST;
    }
    catch (tf2::TransformException &ex) {
        diff_x = 0.0;
        diff_y = 0.0;
        RCLCPP_WARN(this->get_logger(), "Could not transform 'fcu' to 'aruco_marker_frame': %s", ex.what());
        return;
    }
}

void DroneTrackerController::odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    // geometry_msgs::msg::TransformStamped transform;
    // transform.header.stamp = this->now(); 
    // transform.header.frame_id = "ned";
    // transform.child_frame_id  = "base_link";
    // transform.transform.translation.x = msg->position[0];
    // transform.transform.translation.y = msg->position[1];
    // transform.transform.translation.z = msg->position[2];
    // transform.transform.rotation.w = msg->q[0];
    // transform.transform.rotation.x = msg->q[1];
    // transform.transform.rotation.y = msg->q[2];
    // transform.transform.rotation.z = msg->q[3];
    // tf_broadcaster_->sendTransform(transform);
    _vehicle_position_ned = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    _vehicle_orientation = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    _vehicle_velocity_ned = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    _last_odometry_stamp = this->now();
    
    base_x = msg->position[0];
    base_y = msg->position[1];
    base_z = msg->position[2];
}

void DroneTrackerController::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

// void DroneTrackerController::publish_trajectory_setpoint()
// {
//     offboard_setpoint_counter_++;
//     bool offboard_now = (c_mode.flag_control_offboard_enabled == 1);
//     if (offboard_now && !last_offboard_state_) {
//         reference_z_ = base_z;
//         reference_z_captured_ = true;
//         RCLCPP_INFO(this->get_logger(), "Latched reference altitude: %.2f m", reference_z_);
//     }
//     last_offboard_state_ = offboard_now;
//     auto msg = px4_msgs::msg::TrajectorySetpoint();
//     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//     float z_setpoint = reference_z_captured_ ? reference_z_ : HEIGHT;
//     msg.position = {static_cast<float>(_tag.position.x()), static_cast<float>(_tag.position.y()), z_setpoint};
//     msg.yaw = std::numeric_limits<float>::quiet_NaN();;
//     RCLCPP_INFO(this->get_logger(), "Publishing trajectory setpoint: [%.2f, %.2f, %.2f]",_tag.position.x(), _tag.position.y(), z_setpoint);
//     // 安全检查和频率控制
//     if (offboard_now && offboard_setpoint_counter_ >= 5) { // 可以适当减小延迟
//         trajectory_setpoint_publisher_->publish(msg);
//         RCLCPP_INFO(this->get_logger(), "Published TrajectorySetpoint: [%.2f, %.2f, %.2f]",
//                 msg.position[0], msg.position[1], msg.position[2]);
//         offboard_setpoint_counter_ = 0; // 重置计数器
//     }
// }

void DroneTrackerController::publish_trajectory_setpoint(float x, float y, float z)
{
    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = {static_cast<float>(x), static_cast<float>(y), z};
    msg.yaw = std::numeric_limits<float>::quiet_NaN();
    trajectory_setpoint_publisher_->publish(msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                "Published TrajectorySetpoint: [%.2f, %.2f, %.2f]",
                msg.position[0], msg.position[1], msg.position[2]);
}

void DroneTrackerController::publish_full_trajectory_setpoint(float vx, float vy, float vz,
                                                              float ax, float ay, float az)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    const float NaN = std::numeric_limits<float>::quiet_NaN();
    msg.position = {NaN, NaN, NaN};

    msg.velocity = {vx, vy, vz};

    // 只用加速度（前馈/补偿）
    msg.acceleration = {ax, ay, az};

    // yaw 不用：NaN
    msg.yaw = NaN;

    trajectory_setpoint_publisher_->publish(msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 200,
                "Published TrajectorySetpoint - Vel:[%.3f, %.3f, %.3f], Accel:[%.3f, %.3f, %.3f]",
                 vx, vy, vz, ax, ay, az);
}

void DroneTrackerController::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

std::string DroneTrackerController::getStateName(State state)
{
    switch (state) {
        case State::IDLE:
            return "IDLE";
        case State::ARMING:
            return "ARMING";
        case State::HOLDING:
            return "HOLDING";
        case State::TRACKING:
            return "TRACKING";
        case State::DESCEND:
            return "DESCEND";
        default:
            return "UNKNOWN";
    }
}

// ✅ 新增：加速度低通滤波方法
Eigen::Vector3d DroneTrackerController::getFilteredAcceleration(const Eigen::Vector3d& raw_accel)
{
    // 一阶低通滤波器
    // f(k) = α·x(k) + (1-α)·f(k-1)
    // α 越小，滤波越强（但延迟越大）
    // α = 0.3 是一个好的平衡点
    _filtered_accel = _accel_filter_alpha * raw_accel + 
                      (1.0 - _accel_filter_alpha) * _filtered_accel;
    
    return _filtered_accel;
}

void DroneTrackerController::switchToState(State state)
{
    RCLCPP_INFO(this->get_logger(), "Switching state from %s to %s",
                getStateName(current_state).c_str(),
                getStateName(state).c_str());
    current_state = state;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // auto drone_tracker_node = std::make_shared<DroneTrackerController>();
    
    // // 2. 创建一个多线程执行器
    // rclcpp::executors::MultiThreadedExecutor executor;
    
    // // 3. 将你的节点添加到执行器中
    // executor.add_node(drone_tracker_node);
    
    // // 4. 启动执行器，它会管理所有线程并调用回调
    // executor.spin();
    rclcpp::spin(std::make_shared<DroneTrackerController>());
    rclcpp::shutdown();
    return 0;
}