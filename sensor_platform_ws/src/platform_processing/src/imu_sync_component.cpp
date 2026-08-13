
//=============================================
//==========CALLED BY system.launch.py=========
//==============================================


// Called like a plugin with the apellation : platform_processing::ImuSyncComponent
// Goal of the file is to synchronise the important topics of the IMU and also of the camera for the Olive sensors
// We also inject the covariance measurement get by error measurement with AllanVariance or from the datasheet


// A lot of the code is commented, you can uncomment depending of your necessity but Watchout CPU consommation



#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace platform_processing
{
class ImuSyncComponent : public rclcpp::Node
{
public:
  explicit ImuSyncComponent(const rclcpp::NodeOptions & options)
  : Node("imu_sync_node", options)
  {

    
    // Params declaration
    this->declare_parameter("variance_orientation_rp", 0.000076); // 0.5° RMSE in datasheet (page3->metric->Attitude Accuracy) -> 0.5°*pi/180 = error in rad -> (error in rad)**2 = var = 0.000076
    this->declare_parameter("variance_orientation_y", 0.0012);    // < 2° RMSE -> same that above
    this->declare_parameter("variance_angular_velocity", 0.000177); // Gyroscopes = 0.013307796^2  = 0.000177
    this->declare_parameter("variance_linear_acceleration", 0.0000000391607766); // Accelerometers = (0.00019789082)^2 =0.0000000391607766
    
    var_ori_rp_ = this->get_parameter("variance_orientation_rp").as_double();
    var_ori_y_ = this->get_parameter("variance_orientation_y").as_double();
    var_ang_vel_ = this->get_parameter("variance_angular_velocity").as_double();
    var_lin_acc_ = this->get_parameter("variance_linear_acceleration").as_double();
    
    // ==============================================================================
    // 0. Main topics
    // ==============================================================================
    pub_imu_fused_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/fused", rclcpp::SensorDataQoS());
    
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/olive/olixSense/x1/oliveimu/imu", rclcpp::SensorDataQoS(), 
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {
          this->imuCallback(std::move(msg));
      });

    pub_cam_synced_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/olivecam/image/compressed", rclcpp::SensorDataQoS());
    
    sub_cam_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/olive/camera/olivecam/image/compressed", rclcpp::SensorDataQoS(), 
      [this](sensor_msgs::msg::CompressedImage::UniquePtr msg) {
          this->camCallback(std::move(msg));
      });
      
    // ==============================================================================
    // REALSENSE : INFRA 1 & INFRA 2 
    // ==============================================================================
    //pub_rs_infra1_synced_ = this->create_publisher<sensor_msgs::msg::Image>("/d435/infra1_synced", rclcpp::SensorDataQoS());
    //sub_rs_infra1_ = this->create_subscription<sensor_msgs::msg::Image>(
    //  "/camera/d435_node/infra1/image_rect_raw", 10,
    //  [this](sensor_msgs::msg::Image::UniquePtr msg) {
    //    msg->header.stamp = applyCamOffset(msg->header.stamp); // Algned with Jetson timestamp
    //    pub_rs_infra1_synced_->publish(std::move(msg));
    //  });

    //pub_rs_infra2_synced_ = this->create_publisher<sensor_msgs::msg::Image>("/d435/infra2_synced", rclcpp::SensorDataQoS());
    //sub_rs_infra2_ = this->create_subscription<sensor_msgs::msg::Image>(
    //  "/camera/d435_node/infra2/image_rect_raw", 10,
    //  [this](sensor_msgs::msg::Image::UniquePtr msg) {
    //    msg->header.stamp = applyCamOffset(msg->header.stamp); // Algned with Jetson timestamp
    //    pub_rs_infra2_synced_->publish(std::move(msg));
    //  });

    // ==============================================================================
    // 1. CAMERA : POSE & TWIST
    // ==============================================================================
    //pub_cam_pose_synced_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/olivecam/pose_synced", rclcpp::SensorDataQoS());
    //sub_cam_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    //  "/olive/camera/olivecam/pose", rclcpp::SensorDataQoS(), 
    //  [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) {
     //   msg->header.stamp = applyCamOffset(msg->header.stamp);
     //   msg->header.frame_id = "olivecam_imu_frame"; 
     //   pub_cam_pose_synced_->publish(std::move(msg)); 
     // });

    //pub_cam_twist_synced_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/olivecam/twist_synced", rclcpp::SensorDataQoS());
    //sub_cam_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
   //   "/olive/camera/olivecam/twist", rclcpp::SensorDataQoS(), 
    //  [this](geometry_msgs::msg::TwistStamped::UniquePtr msg) {
     //   msg->header.stamp = applyCamOffset(msg->header.stamp);
    //    msg->header.frame_id = "olivecam_imu_frame"; 
    //    pub_cam_twist_synced_->publish(std::move(msg)); 
     // });

    // ==============================================================================
    // 2. CAMERA : ACCELERATION & AHRS
    // ==============================================================================
    //pub_cam_acc_synced_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("/olivecam/linear_acc_synced", rclcpp::SensorDataQoS());
    //sub_cam_acc_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
    //  "/olive/camera/olivecam/linear_acc", rclcpp::SensorDataQoS(), 
    //  [this](geometry_msgs::msg::AccelStamped::UniquePtr msg) {
    //    msg->header.stamp = applyCamOffset(msg->header.stamp); 
     //   msg->header.frame_id = "olivecam_imu_frame"; 
    //    pub_cam_acc_synced_->publish(std::move(msg)); 
    //  });

    pub_cam_ahrs_synced_ = this->create_publisher<sensor_msgs::msg::Imu>("/olivecam/filtered_ahrs_synced", rclcpp::SensorDataQoS());
    sub_cam_ahrs_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/olive/camera/olivecam/filtered_ahrs", rclcpp::SensorDataQoS(), 
        [this](sensor_msgs::msg::Imu::UniquePtr msg) {
        msg->header.stamp = applyCamOffset(msg->header.stamp); 
        msg->header.frame_id = "olivecam_imu_frame";    
        pub_cam_ahrs_synced_->publish(std::move(msg)); 
      });

    // ==============================================================================
    // 3. CAMERA : INFO 
    // ==============================================================================
    //pub_cam_info_synced_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/olivecam/camera_info_synced", rclcpp::SensorDataQoS());
    //sub_cam_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    //  "/olive/camera/olivecam/image/camera_info", rclcpp::SensorDataQoS(), 
    //    [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
    //    msg->header.stamp = applyCamOffset(msg->header.stamp); 
    //    msg->header.frame_id = "olivecam_optical_frame"; 
    //    pub_cam_info_synced_->publish(std::move(msg)); 
    //  });

    // ==============================================================================
    // 4. IMU : ACCELERATION & SPEED
    // ==============================================================================
    //pub_imu_acc_synced_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("/imu/acceleration_synced", rclcpp::SensorDataQoS());
    //sub_imu_acc_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
    //  "/olive/olixSense/x1/oliveimu/acceleration", rclcpp::SensorDataQoS(), 
    //    [this](geometry_msgs::msg::AccelStamped::UniquePtr msg) {
    //    msg->header.stamp = applyImuOffset(msg->header.stamp); 
    //    msg->header.frame_id = "imu_link"; 
    //    pub_imu_acc_synced_->publish(std::move(msg)); 
    //  });

    //pub_imu_vel_synced_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/imu/velocity_synced", rclcpp::SensorDataQoS());
    //sub_imu_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    //  "/olive/olixSense/x1/oliveimu/velocity", rclcpp::SensorDataQoS(),
    //    [this](geometry_msgs::msg::TwistStamped::UniquePtr msg) {
    //    msg->header.stamp = applyImuOffset(msg->header.stamp); 
    //    msg->header.frame_id = "imu_link"; 
    //    pub_imu_vel_synced_->publish(std::move(msg)); 
    //  });
      
    // ==============================================================================
    // TEST build topic imu raw for olivCam
    // ==============================================================================
    
    //pub_cam_raw_imu_synced_ = this->create_publisher<sensor_msgs::msg::Imu>(
    //  "/olivecam/imu_raw_synced", rclcpp::SensorDataQoS());

    // Initializing message_filters subscribers
    //rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    //mf_sub_cam_acc_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::AccelStamped>>(
    //  this, "/olive/camera/olivecam/linear_acc", qos_profile);
      
    //mf_sub_cam_twist_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(
    //  this, "/olive/camera/olivecam/twist", qos_profile);

    // Initializing the synchronizer (Queue size = 10)
    //sync_raw_imu_ = std::make_shared<message_filters::Synchronizer<SyncPolicyRawImu>>(
    //  SyncPolicyRawImu(10), *mf_sub_cam_acc_, *mf_sub_cam_twist_);
      
    // Set the callback that will be triggered only when the two messages are matched
    //sync_raw_imu_->registerCallback(
    //  std::bind(&ImuSyncComponent::rawImuMergeCallback, this, std::placeholders::_1, std::placeholders::_2));  
      

    RCLCPP_INFO(this->get_logger(), "Active synchronization node: Static offsets locked at the first message.");
  }

private:
  // TEST (topic imu raw for olivCam):
  
  // synchronisation
  //typedef message_filters::sync_policies::ApproximateTime<
  //  geometry_msgs::msg::AccelStamped, 
  //  geometry_msgs::msg::TwistStamped> SyncPolicyRawImu;

  // subscribing
  //std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::AccelStamped>> mf_sub_cam_acc_;
  //std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>> mf_sub_cam_twist_;
  
  // The Synchronizer
  //std::shared_ptr<message_filters::Synchronizer<SyncPolicyRawImu>> sync_raw_imu_;
  // Le nouveau Publisher pour l'IMU brut combiné
  //rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_cam_raw_imu_synced_;
  
  //void rawImuMergeCallback(
  //  const geometry_msgs::msg::AccelStamped::ConstSharedPtr& acc_msg, 
  //  const geometry_msgs::msg::TwistStamped::ConstSharedPtr& twist_msg)
  //{
  //  auto raw_imu_msg = sensor_msgs::msg::Imu();

    // 1. Timestamp
  //  raw_imu_msg.header.stamp = applyCamOffset(acc_msg->header.stamp);
  //  raw_imu_msg.header.frame_id = "olivecam_imu_frame";

    // 2. data extraction
  //  constexpr double GRAVITY = 1; //9.80665;
  //  raw_imu_msg.linear_acceleration.x = acc_msg->accel.linear.x * GRAVITY;
  //  raw_imu_msg.linear_acceleration.y = acc_msg->accel.linear.y * GRAVITY;
  //  raw_imu_msg.linear_acceleration.z = acc_msg->accel.linear.z * GRAVITY;
  //  raw_imu_msg.angular_velocity = twist_msg->twist.angular;

    // 3. Prevent zero quaternion
  //  raw_imu_msg.orientation_covariance[0] = -1.0;

    // 4. Covariance injection
  //  raw_imu_msg.angular_velocity_covariance[0] = var_ang_vel_;
  //  raw_imu_msg.angular_velocity_covariance[4] = var_ang_vel_;
  //  raw_imu_msg.angular_velocity_covariance[8] = var_ang_vel_;
    
  //  raw_imu_msg.linear_acceleration_covariance[0] = var_lin_acc_;
  //  raw_imu_msg.linear_acceleration_covariance[4] = var_lin_acc_;
  //  raw_imu_msg.linear_acceleration_covariance[8] = var_lin_acc_;

  //  pub_cam_raw_imu_synced_->publish(raw_imu_msg);
  //}

 
  double var_ori_rp_, var_ori_y_, var_ang_vel_, var_lin_acc_;

  bool imu_offset_initialized_ = false;
  rclcpp::Duration imu_time_offset_{0, 0};

  bool cam_offset_initialized_ = false;
  rclcpp::Duration cam_time_offset_{0, 0};

  // Exponentially Weighted Average
  // Filtering the signal from the USB transmission, digital low-pass filter
  // Time stability preventing form abrupt time jumps with every received frame
  // Stability for kalman filter (UKF or EKF or ...)
  rclcpp::Time applyImuOffset(const builtin_interfaces::msg::Time & stamp)
  {
    rclcpp::Time msg_time(stamp);
    rclcpp::Duration new_offset = this->now() - msg_time;

    if (!imu_offset_initialized_) {
      imu_time_offset_ = new_offset;
      imu_offset_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial IMU time offset locked");
    } else {
      constexpr double alpha = 0.01; // Smoothing factor
      imu_time_offset_ = rclcpp::Duration::from_nanoseconds(
        (1.0 - alpha) * imu_time_offset_.nanoseconds() + alpha * new_offset.nanoseconds());
    }
    return msg_time + imu_time_offset_;
  }

  rclcpp::Time applyCamOffset(const builtin_interfaces::msg::Time & stamp)
  {
    rclcpp::Time msg_time(stamp);
    rclcpp::Duration new_offset = this->now() - msg_time;

    if (!cam_offset_initialized_) {
      cam_time_offset_ = new_offset;
      cam_offset_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial camera time offset locked");
    } else {
      constexpr double alpha = 0.01; // Smoothing factor
      cam_time_offset_ = rclcpp::Duration::from_nanoseconds(
        (1.0 - alpha) * cam_time_offset_.nanoseconds() + alpha * new_offset.nanoseconds());
    }
    return msg_time + cam_time_offset_;
  }

  // --- MAIN CALLBACKS ---
  void imuCallback(sensor_msgs::msg::Imu::UniquePtr msg)
  {
    msg->header.stamp = applyImuOffset(msg->header.stamp);
    msg->header.frame_id  = "imu_link";
    
    double var_ori_rp = this->get_parameter("variance_orientation_rp").as_double();
    double var_ori_y = this->get_parameter("variance_orientation_y").as_double();
    double var_ang_vel = this->get_parameter("variance_angular_velocity").as_double();
    double var_lin_acc = this->get_parameter("variance_linear_acceleration").as_double();
    
    // if the quaternion is zero (norm of 0) :
    if (msg->orientation.w == 0.0 && msg->orientation.x == 0.0 && 
        msg->orientation.y == 0.0 && msg->orientation.z == 0.0) {
        msg->orientation_covariance[0] = -1.0; 
        msg->orientation.w = 1.0;
        msg->orientation.x = 0.0;
        msg->orientation.y = 0.0;
        msg->orientation.z = 0.0;
    } else {
        msg->orientation_covariance[0] = var_ori_rp;
        msg->orientation_covariance[4] = var_ori_rp;
        msg->orientation_covariance[8] = var_ori_y;
    }
    
    msg->angular_velocity_covariance[0] = var_ang_vel;
    msg->angular_velocity_covariance[4] = var_ang_vel;
    msg->angular_velocity_covariance[8] = var_ang_vel;
    msg->linear_acceleration_covariance[0] = var_lin_acc;
    msg->linear_acceleration_covariance[4] = var_lin_acc;
    msg->linear_acceleration_covariance[8] = var_lin_acc;
    
    pub_imu_fused_->publish(std::move(msg));
  }

void camCallback(sensor_msgs::msg::CompressedImage::UniquePtr msg) {
    msg->header.stamp = applyCamOffset(msg->header.stamp);
    msg->header.frame_id = "olivecam_optical_frame";
    pub_cam_synced_->publish(std::move(msg));
}

  // --- DECLARATIONS OF PUBLISHERS / SUBSCRIBERS ---
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_fused_;
  
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_cam_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_cam_synced_;
  
  //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rs_infra1_;
  //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rs_infra1_synced_;
  
  //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rs_infra2_;
  //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rs_infra2_synced_;
  
  //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cam_pose_;
  //rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cam_pose_synced_;
  
  //rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cam_twist_;
  //rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cam_twist_synced_;
  
  //rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr sub_cam_acc_;
  //rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_cam_acc_synced_;
  
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_cam_ahrs_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_cam_ahrs_synced_;
  
  //rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;
  //rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info_synced_;

  //rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr sub_imu_acc_;
  //rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_imu_acc_synced_;
  
  //rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_imu_vel_;
  //rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_imu_vel_synced_;
};
} 

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(platform_processing::ImuSyncComponent)
