// #include <cstdio>
// #include "kf_tracker_node.hpp"

// KFTracker::KFTracker(const rclcpp::NodeOptions & options): Node("kf_tracker_node",options),
//   q_(0.1),
//   r_(0.01),
//   dt_pred_(0.05),
//   is_state_initialized_(false),
//   state_buffer_size_(40),
//   do_update_step_(true),
//   measurement_off_time_(2.0),
//   debug_(false)
// {
//   declareAndGetParams();

//   initKF();

//   auto timer_period = std::chrono::duration<double>(dt_pred_);
//   kf_loop_timer_ = this->create_wall_timer(timer_period,std::bind(&KFTracker::filterLoop,this));

//   pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/world_tag_pose",10,std::bind(&KFTracker::poseCallback,this,std::placeholders::_1));

//   state_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf_estimate",10);

// }

// void KFTracker::declareAndGetParams()
// {
//   this->declare_parameter<double>("dt_pred",dt_pred_);
//   this->declare_parameter<double>("q_std", q_);
//   this->declare_parameter<double>("r_std", r_);
//   this->declare_parameter<int>("state_buffer_length", 40);
//   this->declare_parameter<bool>("do_kf_update_step", do_update_step_);
//   this->declare_parameter<double>("measurement_off_time", measurement_off_time_);
//   this->declare_parameter<bool>("print_debug_msg", debug_);

//   this->get_parameter("dt_pred",dt_pred_);
//   this->get_parameter("q_std", q_);
//   this->get_parameter("r_std", r_);
//   int buff_size;
//   this->get_parameter("state_buffer_length", buff_size);
//   state_buffer_size_ = static_cast<unsigned int>(buff_size);
//   this->get_parameter("do_kf_update_step", do_update_step_);
//   this->get_parameter("measurement_off_time", measurement_off_time_);
//   this->get_parameter("print_debug_msg", debug_);

//   RCLCPP_INFO(this->get_logger(), "State buffer length corresponds to %f seconds", dt_pred_ * static_cast<double>(state_buffer_size_));
// }

// void KFTracker::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//   Eigen::Vector3d pos;
//   pos << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
//   if (pos.norm() > 10000.0)
//   {
//     RCLCPP_ERROR(this->get_logger(),"[KF poseCallback] Infinite measurement value. Ignoring measurement.");
//     return;
//   }

//   z_meas_.time_stamp = msg->header.stamp;
//   z_meas_.z(0) = msg->pose.position.x;
//   z_meas_.z(1) = msg->pose.position.y;
//   z_meas_.z(2) = msg->pose.position.z;

//   if (!is_state_initialized_)
//   {
//     kf_state_pred_.x.resize(6,1);
//     kf_state_pred_.x.setZero();
//     kf_state_pred_.x.head(3) = z_meas_.z;

//     is_state_initialized_ = true;
//     RCLCPP_INFO(this->get_logger(),"KF state estimate is initialized.");
//   }
// }

// void KFTracker::publishState(void)
// {
//   geometry_msgs::msg::PoseWithCovarianceStamped msg;
//   msg.header.stamp = kf_state_pred_.time_stamp;
//   msg.pose.pose.position.x = kf_state_pred_.x(0);
//   msg.pose.pose.position.y = kf_state_pred_.x(1);
//   msg.pose.pose.position.z = kf_state_pred_.x(2);
//   msg.pose.pose.orientation.w = 1.0;

//   auto pxx = kf_state_pred_.P(0,0);
//   auto pyy = kf_state_pred_.P(1,1);
//   auto pzz = kf_state_pred_.P(2,2);
//   msg.pose.covariance[0] = pxx;
//   msg.pose.covariance[7] = pyy;
//   msg.pose.covariance[14] = pzz;
//   state_pub_->publish(msg);
// }

// void KFTracker::filterLoop(void)
// {
//    if(is_state_initialized_)
//    {
//       if( (this->get_clock()->now().seconds() - z_meas_.time_stamp.seconds()) > measurement_off_time_)
//       {
//          RCLCPP_ERROR(this->get_logger(), "No measurements received for more than %f seconds. Stopping filter....", measurement_off_time_);
//          is_state_initialized_ = false;
//          initKF();
//          return;
//       }

//       if(!predict())
//          return;

//       if(do_update_step_)
//          update();

//       publishState();
//    }
//    else
//    {
//       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Filter is not initialized. Requires a measurement first.");
//    }
// }

// void KFTracker::setQ(void)
// {
//    // This is for constant velocity model \in R^3

//    Q_.resize(6,6);
//    Q_ = Eigen::MatrixXd::Zero(6,6);
//    Q_(0,0) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // x with x
//    Q_(0,3) = 0.5*dt_pred_*dt_pred_; // x with vx
//    Q_(1,1) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // y with y
//    Q_(1,4) = 0.5*dt_pred_*dt_pred_; // y with vy
//    Q_(2,2) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // z with z
//    Q_(2,5) = 0.5*dt_pred_*dt_pred_; // z with vz
//    Q_(3,0) = Q_(0,3); // vx with x. Symmetric
//    Q_(3, 3) = dt_pred_; // vx with vx
//    Q_(4,1) = Q_(1,4); // vy with y. Symmetric
//    Q_(4,4) = dt_pred_; // vy with vy
//    Q_(5,2) = Q_(2,5); // vz with z. Symmetric
//    Q_(5,5) = dt_pred_; // vz with vz

//    Q_ = q_*q_*Q_; // multiply by process noise variance
// }

// void KFTracker::setR(void)
// {
//    R_.resize(3,3);
//    R_ = Eigen::MatrixXd::Identity(3,3);
//    R_ = r_*r_*R_; // multiply by observation noise variance
// }

// void KFTracker::setF(void)
// {
//    // This is for constant velocity model \in R^3

//    F_.resize(6,6);
//    F_ = Eigen::MatrixXd::Identity(6,6);
//    F_(0,3) = dt_pred_; // x - vx
//    F_(1,4) = dt_pred_; // y - vy
//    F_(2,5) = dt_pred_; // z - vz
// }

// void KFTracker::setH(void)
// {
//    H_.resize(3,6);
//    H_ = Eigen::MatrixXd::Zero(3,6);
//    H_(0,0) = 1.0; // observing x
//    H_(1,1) = 1.0; // observing y
//    H_(2,2) = 1.0; // observing z
// }

// void KFTracker::initP(void)
// {
//    kf_state_pred_.P = Q_;
// }

// void KFTracker::initKF(void)
// {
//    kf_state_pred_.time_stamp = this->get_clock()->now();
//    kf_state_pred_.x = Eigen::MatrixXd::Zero(6,1);
//    kf_state_pred_.x(3,0) = 1e-7; kf_state_pred_.x(4,0) = 1e-7; kf_state_pred_.x(5,0) = 1e-7;
//    setQ(); initP(); setR(); setF(); setH();
//    state_buffer_.clear();
//    z_meas_.time_stamp = this->get_clock()->now();
//    z_meas_.z.resize(3,1); 
//    z_meas_.z = Eigen::MatrixXd::Zero(3,1);
//    z_last_meas_ = z_meas_;
//    RCLCPP_INFO(this->get_logger(), "KF is initialized.");
// }

// void KFTracker::updateStateBuffer(void)
// {
//    kf_state kfstate;
//    kfstate.time_stamp = kf_state_pred_.time_stamp;
//    kfstate.x = kf_state_pred_.x;
//    kfstate.P = kf_state_pred_.P;
//    state_buffer_.push_back(kfstate);
//    if(state_buffer_.size() > state_buffer_size_)
//       state_buffer_.erase(state_buffer_.begin()); // remove first element in the buffer
// }

// bool KFTracker::predict(void)
// {
//    kf_state_pred_.x = F_*kf_state_pred_.x;
//    if(kf_state_pred_.x.norm() > 10000.0)
//    {
//       RCLCPP_ERROR(this->get_logger(), "State prediction exploded!!!");
//       is_state_initialized_ = false;
//       initKF();
//       return false;
//    }
//    kf_state_pred_.P = F_*kf_state_pred_.P*F_.transpose() + Q_;
//    kf_state_pred_.time_stamp = this->get_clock()->now();
//    updateStateBuffer();
//    return true;
// }

// void KFTracker::update(void)
// {
//    sensor_measurement current_z; // store the current measurement in seperate variable as the original one is continuously updated
//    current_z.time_stamp = z_meas_.time_stamp;
//    current_z.z = z_meas_.z;

//    // check if we got new measurement
//    if (current_z.time_stamp.seconds() == z_last_meas_.time_stamp.seconds())
//    {
//       if(debug_)
//          RCLCPP_WARN(this->get_logger(), "No new measurment. Skipping KF update step.");
//       return;
//    }

//    // Make sure the current measurement time is not ahead of current prediction
//    if(current_z.time_stamp.seconds() > state_buffer_.back().time_stamp.seconds())
//    {
//       if(debug_)
//          RCLCPP_WARN(this->get_logger(),"Measurement is ahead of prediction. Skipping KF update step.");
//       return;
//    }

//    bool state_found = false;
//    if(state_buffer_.size() > 1)
//    {
//       // Find closest prediction time to the current measurement. Then, use it for update.
//       //for (std::vector<kf_state>::iterator it = state_buffer_.end() ; it != state_buffer_.begin(); it--)
//       for (size_t i=0; i < state_buffer_.size(); i++)
//       {
//          auto dt = state_buffer_[i].time_stamp.seconds() - current_z.time_stamp.seconds();
//          if( dt > dt_pred_ and dt < 2*dt_pred_)
//          {
//             kf_state_pred_.time_stamp = state_buffer_[i].time_stamp;
//             kf_state_pred_.x = state_buffer_[i].x;
//             kf_state_pred_.P = state_buffer_[i].P;
//             state_buffer_.erase(state_buffer_.begin(),state_buffer_.begin()+i+1); // remove old states
//             state_found = true;
//             break;
//          }
//       } // end loop over state_buffer_
//    }

//    if(state_found)//do KF update step
//    {
//       // Following Wikipedia KF convention

//       // compute innovation
//       auto y = current_z.z - H_*kf_state_pred_.x;

//       // Innovation covariance
//       auto S = H_*kf_state_pred_.P*H_.transpose() + R_;

//       // Kalman gain
//       auto K = kf_state_pred_.P*H_.transpose()*S.inverse();

//       // Updated state estimate and its covariance
//       kf_state_pred_.x = kf_state_pred_.x + K*y;
//       kf_state_pred_.P = kf_state_pred_.P - K*H_*kf_state_pred_.P;

//       // For debugging
//       if(debug_)
//          RCLCPP_INFO(this->get_logger(), "KF update step is executed.");

//       // z_last_meas_.time_stamp = z.time_stamp;
//       // z_last_meas_.z = z.z;
//       z_last_meas_.time_stamp = current_z.time_stamp;
//       z_last_meas_.z = current_z.z;

//       //std::cout << "Difference between current time and accepted measurement: " << (ros::Time::now().toSec() - current_z.time_stamp.toSec()) / dt_pred_ << std::endl;

//       // Project state estimate for the remaining time
//       int N = (int) (this->get_clock()->now().seconds() - current_z.time_stamp.seconds());
//       for (int i=0; i<N; i++)
//          predict();
//    }
//    else
//    {
//       if(debug_)
//          RCLCPP_WARN(this->get_logger(), "Measurement is too old to be used. Skipping KF update step.");
//    }
// }

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions options;
//   auto node = std::make_shared<KFTracker>(options);
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


#include "kf_tracker/kf_tracker_lib.hpp"

// 构造函数：接收参数并初始化
KFTrackerCore::KFTrackerCore(rclcpp::Logger logger, double q_std, double r_std, double dt_pred) :
  logger_(logger),
  q_(q_std),
  r_(r_std),
  dt_pred_(dt_pred),
  is_state_initialized_(false),
  state_buffer_size_(40),
  debug_(true)  // 启用调试模式以便诊断问题
{
    initKF();
    RCLCPP_INFO(logger_, "KFTrackerCore instance created with dt_pred=%.4f, q=%.4f, r=%.4f", dt_pred_, q_, r_);
}

bool KFTrackerCore::isInitialized() const
{
    return is_state_initialized_;
}

// 主接口函数，替代了原来的 poseCallback 和 filterLoop
void KFTrackerCore::updateWithMeasurement(const geometry_msgs::msg::PoseStamped::SharedPtr measurement)
{
    // 1. 更新测量值 (来自旧的 poseCallback)
    z_meas_.time_stamp = measurement->header.stamp;
    z_meas_.z(0) = measurement->pose.position.x;
    z_meas_.z(1) = measurement->pose.position.y;
    z_meas_.z(2) = measurement->pose.position.z;

    // 2. 首次初始化 (来自旧的 poseCallback)
    if (!is_state_initialized_)
    {
        kf_state_pred_.x.resize(6,1);
        kf_state_pred_.x.setZero();
        kf_state_pred_.x.head(3) = z_meas_.z;
        kf_state_pred_.time_stamp = z_meas_.time_stamp;
        z_last_meas_ = z_meas_;
        is_state_initialized_ = true;
        updateStateBuffer(); // 保存初始状态到 buffer
        RCLCPP_INFO(logger_, "KF state estimate is initialized with position: [%.2f, %.2f, %.2f]", 
                    z_meas_.z(0), z_meas_.z(1), z_meas_.z(2));
        return; // 第一次只初始化，不进行滤波
    }

    // 3. 执行滤波循环 (来自旧的 filterLoop)
   //  ✅ 修复：只在状态缓冲有数据时进行预测
   //  这样可以避免多次应用常速度模型导致的速度膨胀
   //  if(state_buffer_.size() > 0)
   //  {
   //      // 状态缓冲已有数据，进行一次预测
   //      if(!predict()) return;
   //  }
   //  // 执行测量更新
   if(!predict()) return;
    update();
}

// 获取结果的函数，替代了 publishState
geometry_msgs::msg::PoseWithCovarianceStamped KFTrackerCore::getFilteredState() const
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = kf_state_pred_.time_stamp;
  msg.header.frame_id = "ned";
  msg.pose.pose.position.x = kf_state_pred_.x(0);
  msg.pose.pose.position.y = kf_state_pred_.x(1);
  msg.pose.pose.position.z = kf_state_pred_.x(2);
  msg.pose.pose.orientation.w = 1.0;
  // ... 填充协方差 ...
  msg.pose.covariance[0] = kf_state_pred_.P(0,0);
  msg.pose.covariance[7] = kf_state_pred_.P(1,1);
  msg.pose.covariance[14] = kf_state_pred_.P(2,2);
  return msg;
}

// =========================================================================
// == 以下是几乎不变的内部数学逻辑 (除了时间和日志的修改) ==
// =========================================================================

void KFTrackerCore::initKF()
{
   // 注意：不再能访问 this->get_clock()，所以初始时间戳可以设为零或由外部传入
   kf_state_pred_.time_stamp = rclcpp::Time(0);
   kf_state_pred_.x = Eigen::MatrixXd::Zero(6,1);
   kf_state_pred_.P = Eigen::MatrixXd::Identity(6,6);
   
   // 初始化测量向量
   z_meas_.time_stamp = rclcpp::Time(0);
   z_meas_.z.resize(3,1);
   z_meas_.z.setZero();
   
   // 初始化矩阵
   setQ(); 
   initP(); 
   setR(); 
   setF(); 
   setH();
   
   // 清空 buffer
   state_buffer_.clear();
   
   // 初始化上一次测量
   z_last_meas_ = z_meas_;
   
   RCLCPP_INFO(logger_, "KF initialization done. Q norm=%.6f, R norm=%.6f", Q_.norm(), R_.norm());
}

bool KFTrackerCore::predict()
{
   // kf_state_pred_.x = F_*kf_state_pred_.x;
   // if(kf_state_pred_.x.norm() > 10000.0)
   // {
   //    RCLCPP_ERROR(logger_, "State prediction exploded!!!");
   //    is_state_initialized_ = false;
   //    initKF();
   //    return false;
   // }
   // kf_state_pred_.P = F_*kf_state_pred_.P*F_.transpose() + Q_;
   
   // //使用当前测量的时间戳作为预测状态的时间
   // // 这样状态缓冲中的时间戳与实际物理时间对齐
   // kf_state_pred_.time_stamp = z_meas_.time_stamp; 
   // updateStateBuffer();
   
   // if(debug_)
   // {
   //    RCLCPP_INFO(logger_, "[KF PREDICT] x=%.3f, y=%.3f, z=%.3f | vx=%.4f, vy=%.4f, vz=%.4f | buf_size=%zu",
   //                kf_state_pred_.x(0), kf_state_pred_.x(1), kf_state_pred_.x(2),
   //                kf_state_pred_.x(3), kf_state_pred_.x(4), kf_state_pred_.x(5),
   //                state_buffer_.size());
   // }
   // return true;
   // 改进：使用实际时间差而不是固定步长
   // 这避免了多步累积导致位置过度推进的问题
    
   double cur_t = kf_state_pred_.time_stamp.seconds();
   double meas_t = z_meas_.time_stamp.seconds();
   double dt_actual = meas_t - cur_t;
   
   // 若测量时间不比当前预测时间更新，无需预测
   if (dt_actual <= 0.0)
   {
      if(debug_)
         RCLCPP_INFO(logger_, "[KF PREDICT] Skipped (dt_actual=%.6f <= 0)", dt_actual);
      return true;
   }
   
   // 核心改进：用实际的 dt_actual 而不是固定的 dt_pred_
   // 这样预测的位移 = v * dt_actual，更精确
   // 但为了保持 Q 矩阵的一致性，我们仍然用 dt_pred_ 来缩放 Q
   
   // 创建临时的 F 矩阵，用实际的 dt_actual
   Eigen::MatrixXd F_actual = Eigen::MatrixXd::Identity(6, 6);
   F_actual(0, 3) = dt_actual;
   F_actual(1, 4) = dt_actual;
   F_actual(2, 5) = dt_actual;
   
   // 创建与实际 dt 相符的 Q
   Eigen::MatrixXd Q_actual = Eigen::MatrixXd::Zero(6, 6);
   Q_actual(0,0) = 1./3.*dt_actual*dt_actual*dt_actual;
   Q_actual(0,3) = 0.5*dt_actual*dt_actual;
   Q_actual(1,1) = 1./3.*dt_actual*dt_actual*dt_actual;
   Q_actual(1,4) = 0.5*dt_actual*dt_actual;
   Q_actual(2,2) = 1./3.*dt_actual*dt_actual*dt_actual;
   Q_actual(2,5) = 0.5*dt_actual*dt_actual;
   Q_actual(3,0) = Q_actual(0,3);
   Q_actual(3,3) = dt_actual;
   Q_actual(4,1) = Q_actual(1,4);
   Q_actual(4,4) = dt_actual;
   Q_actual(5,2) = Q_actual(2,5);
   Q_actual(5,5) = dt_actual;
   Q_actual = q_*q_*Q_actual;
   
   // 执行单步预测（使用实际 dt）
   kf_state_pred_.x = F_actual * kf_state_pred_.x;
   if (kf_state_pred_.x.norm() > 10000.0)
   {
      RCLCPP_ERROR(logger_, "State prediction exploded!!!");
      is_state_initialized_ = false;
      initKF();
      return false;
   }
   kf_state_pred_.P = F_actual * kf_state_pred_.P * F_actual.transpose() + Q_actual;
   
   // 更新时间戳到测量时刻
   kf_state_pred_.time_stamp = z_meas_.time_stamp;
   updateStateBuffer();
   
   if(debug_)
   {
      RCLCPP_INFO(logger_, "[KF PREDICT] dt_actual=%.6f, x=%.3f, y=%.3f, z=%.3f | vx=%.4f, vy=%.4f, vz=%.4f | buf_size=%zu",
                  dt_actual,
                  kf_state_pred_.x(0), kf_state_pred_.x(1), kf_state_pred_.x(2),
                  kf_state_pred_.x(3), kf_state_pred_.x(4), kf_state_pred_.x(5),
                  state_buffer_.size());
   }
   return true;
}

// ... 将你原来的 setQ, setR, setF, setH, initP, update, updateStateBuffer 等函数
// ... 完整地复制到这里。确保把所有的 RCLCPP_... 日志调用的第一个参数
// ... 从 this->get_logger() 改为 logger_。
// ... 并且把 this->get_clock()->now() 的调用都移除或替换掉。
void KFTrackerCore::setQ(void)
{
   // This is for constant velocity model \in R^3

   Q_.resize(6,6);
   Q_ = Eigen::MatrixXd::Zero(6,6);
   Q_(0,0) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // x with x
   Q_(0,3) = 0.5*dt_pred_*dt_pred_; // x with vx
   Q_(1,1) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // y with y
   Q_(1,4) = 0.5*dt_pred_*dt_pred_; // y with vy
   Q_(2,2) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // z with z
   Q_(2,5) = 0.5*dt_pred_*dt_pred_; // z with vz
   Q_(3,0) = Q_(0,3); // vx with x. Symmetric
   Q_(3, 3) = dt_pred_; // vx with vx
   Q_(4,1) = Q_(1,4); // vy with y. Symmetric
   Q_(4,4) = dt_pred_; // vy with vy
   Q_(5,2) = Q_(2,5); // vz with z. Symmetric
   Q_(5,5) = dt_pred_; // vz with vz

   Q_ = q_*q_*Q_; // multiply by process noise variance
}

void KFTrackerCore::setR(void)
{
   R_.resize(3,3);
   R_ = Eigen::MatrixXd::Identity(3,3);
   R_ = r_*r_*R_; // multiply by observation noise variance
}

void KFTrackerCore::setF(void)
{
   // This is for constant velocity model \in R^3

   F_.resize(6,6);
   F_ = Eigen::MatrixXd::Identity(6,6);
   F_(0,3) = dt_pred_; // x - vx
   F_(1,4) = dt_pred_; // y - vy
   F_(2,5) = dt_pred_; // z - vz
}

void KFTrackerCore::setH(void)
{
   H_.resize(3,6);
   H_ = Eigen::MatrixXd::Zero(3,6);
   H_(0,0) = 1.0; // observing x
   H_(1,1) = 1.0; // observing y
   H_(2,2) = 1.0; // observing z
}

void KFTrackerCore::initP(void)
{
   kf_state_pred_.P = Q_;
}


void KFTrackerCore::update(void)
{
   sensor_measurement current_z; // store the current measurement in seperate variable as the original one is continuously updated
   current_z.time_stamp = z_meas_.time_stamp;
   current_z.z = z_meas_.z;

   // check if we got new measurement
   if (current_z.time_stamp.seconds() == z_last_meas_.time_stamp.seconds())
   {
      if(debug_)
         RCLCPP_WARN(logger_, "No new measurment. Skipping KF update step.");
      return;
   }

   bool state_found = false;
   int matched_idx = -1;
   
   // 查找与当前测量时间戳最接近的预测状态
   if(state_buffer_.size() > 0)
   {
      double min_time_diff = std::numeric_limits<double>::infinity();
      
      for (size_t i = 0; i < state_buffer_.size(); i++)
      {
         double time_diff = std::fabs(state_buffer_[i].time_stamp.seconds() - current_z.time_stamp.seconds());
         
         if(debug_)
         {
            RCLCPP_INFO(logger_, "[UPDATE] Buffer[%zu] time=%.4f, measurement time=%.4f, diff=%.4f",
                        i, state_buffer_[i].time_stamp.seconds(), current_z.time_stamp.seconds(), time_diff);
         }
         
         // 找到最接近的状态（时间差最小）
         if(time_diff < min_time_diff && time_diff < dt_pred_ * 2.0)
         {
            min_time_diff = time_diff;
            matched_idx = i;
            state_found = true;
         }
      }
   }

   if(state_found && matched_idx >= 0)
   {
      // if(debug_)
      // {
      //    RCLCPP_INFO(logger_, "[UPDATE] Using buffer[%d]: time_diff=%.4f ms, innovation=[%.4f, %.4f, %.4f]", 
      //                matched_idx, 
      //                std::fabs(state_buffer_[matched_idx].time_stamp.seconds() - current_z.time_stamp.seconds()) * 1000.0,
      //                y(0), y(1), y(2));
      // }
      
      // 使用找到的状态进行更新
      kf_state_pred_.time_stamp = state_buffer_[matched_idx].time_stamp;
      kf_state_pred_.x = state_buffer_[matched_idx].x;
      kf_state_pred_.P = state_buffer_[matched_idx].P;
      state_buffer_.erase(state_buffer_.begin(), state_buffer_.begin() + matched_idx + 1); // remove old states
      
      // Following Wikipedia KF convention
      // compute innovation (测量值 - 预测值)
      auto y = current_z.z - H_*kf_state_pred_.x;

      if(debug_)
      {
         RCLCPP_INFO(logger_, "[UPDATE] Innovation: [%.4f, %.4f, %.4f]", y(0), y(1), y(2));
      }

      // Innovation covariance
      auto S = H_*kf_state_pred_.P*H_.transpose() + R_;

      // Kalman gain
      auto K = kf_state_pred_.P*H_.transpose()*S.inverse();

      // Updated state estimate and its covariance
      kf_state_pred_.x = kf_state_pred_.x + K*y;
      kf_state_pred_.P = kf_state_pred_.P - K*H_*kf_state_pred_.P;

      if(debug_)
      {
         RCLCPP_INFO(logger_, "[UPDATE DONE] New: x=%.3f, y=%.3f, z=%.3f | vx=%.4f, vy=%.4f, vz=%.4f",
                     kf_state_pred_.x(0), kf_state_pred_.x(1), kf_state_pred_.x(2),
                     kf_state_pred_.x(3), kf_state_pred_.x(4), kf_state_pred_.x(5));
      }

      z_last_meas_.time_stamp = current_z.time_stamp;
      z_last_meas_.z = current_z.z;
   }
   else
   {
      if(debug_)
         RCLCPP_WARN(logger_, "No matching state found in buffer (size=%zu). Skipping KF update step.", state_buffer_.size());
   }
}

void KFTrackerCore::updateStateBuffer(void)
{
   kf_state kfstate;
   kfstate.time_stamp = kf_state_pred_.time_stamp;
   kfstate.x = kf_state_pred_.x;
   kfstate.P = kf_state_pred_.P;
   state_buffer_.push_back(kfstate);
   if(state_buffer_.size() > state_buffer_size_)
      state_buffer_.erase(state_buffer_.begin()); // remove first element in the buffer
}

Eigen::Vector3d KFTrackerCore::getFilteredVelocity() const
{
    if (is_state_initialized_) {
        // 返回状态向量中的速度部分
        return kf_state_pred_.x.tail<3>(); 
    }
    return Eigen::Vector3d::Zero();
}