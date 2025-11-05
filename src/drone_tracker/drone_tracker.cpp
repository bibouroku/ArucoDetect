#include "drone_tracker.hpp"

using namespace std::chrono_literals;

DroneTrackerController::DroneTrackerController() : Node("drone_tracker_controller"),current_state(State::IDLE)
{
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

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
    offboard_setpoint_counter_ = 0;
    reference_z_captured_ = false;
    last_offboard_state_  = false;

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
    timer_ = this->create_wall_timer(100ms, std::bind(&DroneTrackerController::run_state_machine, this));
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
    if (offboard_setpoint_counter_ > 10 ) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // 切换到OFFBOARD模式
        arm();

        // 检查是否已经切换到Offboard模式
        if (c_mode.flag_control_offboard_enabled) {
            switchToState(State::TRACKING);
        }
    }
    publish_offboard_control_mode();
    offboard_setpoint_counter_++;
}

void DroneTrackerController::run_tracking_state()
{
    publish_offboard_control_mode();
    // 检查是否长时间没有看到二维码
    if ((this->now() - _last_tag_seen_time).seconds() > 2.0) {
        RCLCPP_WARN(this->get_logger(), "Tag lost for too long, holding position.");
        // 丢失目标，悬停在最后看到的位置
        publish_trajectory_setpoint(_last_seen_tag.position.x(), _last_seen_tag.position.y(), HEIGHT);
        return;
    }
    
    // 发布目标位置作为设定点
    publish_trajectory_setpoint(_tag.position.x(), _tag.position.y(), HEIGHT);
    RCLCPP_INFO(this->get_logger(), "TRACKING: Following tag at [%.2f, %.2f, %.2f]",
                _tag.position.x(), _tag.position.y(), HEIGHT);

    // 检查是否满足进入降落状态的条件
    /*if (is_directly_above_target()) {
        RCLCPP_INFO(this->get_logger(), "Directly above target, preparing to descend.");
        _last_seen_tag = _tag; // 锁定降落目标位置
        switchToState(State::DESCEND);
    }*/
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
    tag_camera.timestamp = this->now();
    _tag = getTagWorld(tag_camera);
    _last_tag_seen_time = this->now();
    _last_seen_tag = _tag;
    RCLCPP_INFO(this->get_logger(), "Transform from ned to april tag:");
    RCLCPP_INFO(this->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f",
                _tag.position.x(),
                _tag.position.y(),
                _tag.position.z());

}
DroneTrackerController::ArucoTag DroneTrackerController::getTagWorld(const ArucoTag& tag_camera) {
    Eigen::Matrix3d R;
    R << 0, -1, 0,
        1, 0, 0,
        0, 0, 1;
    Eigen::Quaterniond quat_NED(R);
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
    base_x = msg->position[0];
    base_y = msg->position[1];
    base_z = msg->position[2];
}

void DroneTrackerController::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
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
    RCLCPP_INFO(this->get_logger(), "Published TrajectorySetpoint: [%.2f, %.2f, %.2f]",
                msg.position[0], msg.position[1], msg.position[2]);
    trajectory_setpoint_publisher_->publish(msg);
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
        case State::TRACKING:
            return "TRACKING";
        case State::DESCEND:
            return "DESCEND";
        default:
            return "UNKNOWN";
    }
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