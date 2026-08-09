#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <poll.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <array>
#include <algorithm>

// DroneCAN IDs
#define UAVCAN_EQUIPMENT_OPTICAL_FLOW_ID 20200
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_ID 1050


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <canard.h>
#pragma GCC diagnostic pop

namespace uavcan_bridge {


template<typename T>
class LowPassFilter {
public:
    LowPassFilter(T alpha = 0.2) : alpha_(alpha), initialized_(false), prev_out_(0) {}
    
    T update(T new_val) {
        if (!initialized_) {
            prev_out_ = new_val;
            initialized_ = true;
            return new_val;
        }
        // Équation IIR standard : y(n) = alpha * x(n) + (1 - alpha) * y(n-1)
        prev_out_ = alpha_ * new_val + (1.0f - alpha_) * prev_out_;
        return prev_out_;
    }
    
    void reset() { initialized_ = false; }
private:
    T alpha_;
    bool initialized_;
    T prev_out_;
};


class DronecanBridgeComponent;

//def
bool shouldAcceptTransfer(const CanardInstance* ins, uint64_t* out_data_type_signature, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id);
void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);

// class
class DronecanBridgeComponent : public rclcpp::Node {
public:
    // EMA smoothing parameter (0.0 = frozen, 1.0 = raw data)
    // 0.2 effectively filters out optical noise
    float ema_alpha_ = 0.2f; 
    float z_filtered_ = -1.0f; // Initialisation
    float current_distance_z_ = 0.0f;
    DronecanBridgeComponent(const rclcpp::NodeOptions & options)
    : Node("dronecan_bridge", options) {
        
        this->declare_parameter<std::string>("can_interface", "slcan0");
        auto can_iface = this->get_parameter("can_interface").as_string();

        //initializing publishers
        flow_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/optical_flow/velocity", rclcpp::QoS(10));
        range_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/optical_flow/range", rclcpp::QoS(10));

        canardInit(&canard_, canard_memory_pool_, sizeof(canard_memory_pool_), onTransferReceived, shouldAcceptTransfer, this);
        canardSetLocalNodeID(&canard_, 127); // The Jetson has the ID 127

        if (!init_socketcan(can_iface)) {
            RCLCPP_FATAL(this->get_logger(), "Impossible to open the bus CAN: %s", can_iface.c_str());
            throw std::runtime_error("CAN Init Failed");
        }

        is_running_ = true;
        rx_thread_ = std::thread(&DronecanBridgeComponent::can_rx_loop, this);
        
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/olive/olixSense/x1/oliveimu/imu", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
                double r_imu, p_imu, y_imu;
                tf2::Matrix3x3(q).getRPY(r_imu, p_imu, y_imu);

                // IMU gas a 90 deg rotation in .xacro
                float drone_roll = static_cast<float>(-p_imu);
                float drone_pitch = static_cast<float>(r_imu);

                roll_.store(drone_roll);
                pitch_.store(drone_pitch);
            });

        RCLCPP_INFO(this->get_logger(), "DroneCAN Bridge active on %s (Threaded). Ready for Flow and Range!", can_iface.c_str());
    }

    ~DronecanBridgeComponent() { 
        is_running_ = false;
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if (socket_fd_ >= 0) close(socket_fd_);
    }

    void publish_flow_data(float x, float y, float cov, uint64_t hardware_timestamp_usec) {
        auto msg = geometry_msgs::msg::TwistWithCovarianceStamped();
        
        // time synchro + id + cov
        msg.header.stamp = this->now();
        //msg.header.stamp = rclcpp::Time(hardware_timestamp_usec * 1000ULL, RCL_SYSTEM_TIME);
        msg.header.frame_id = "optical_flow_link";
        msg.twist.twist.linear.x = x;
        msg.twist.twist.linear.y = y;
        msg.twist.covariance[0]  = cov;
        msg.twist.covariance[7]  = cov;
        msg.twist.covariance[14] = 10000.0;
        msg.twist.covariance[21] = 10000.0;
        msg.twist.covariance[28] = 10000.0;
        msg.twist.covariance[35] = 10000.0;       
        flow_pub_->publish(msg);
        (void)hardware_timestamp_usec;
    }
    

    void publish_range_data(float distance, uint64_t hardware_timestamp_usec) {
        float r = this->roll_.load();
        float p = this->pitch_.load();
        float z_sensor = distance * cosf(r) * cosf(p);
        // H-Flow offset by the URDF : X = 0.17, Z = -0.0225
        float z_base_link = z_sensor + (0.17f * sinf(p)) + (0.0225f * cosf(r) * cosf(p));
        
        float z_filtered = range_filter_.update(z_base_link);
        
        // Updating current altitude for speed calculation (line ~310)
        current_distance_z_ = z_filtered;
        
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.pose.pose.orientation.w = 1.0;// quaternion (0,0,0,0) do crash the UKF so we give him an orientation at the start
        msg.header.stamp = this->now(); // time synchro + id + cov
        //msg.header.stamp = rclcpp::Time(hardware_timestamp_usec * 1000ULL, RCL_SYSTEM_TIME);
        msg.header.frame_id = "odom"; // or "optical_flow_link"; // base_link
        msg.pose.pose.position.z = z_filtered; 
        msg.pose.covariance[0]  = 10000.0;
        msg.pose.covariance[7]  = 10000.0;
        msg.pose.covariance[14] = 0.001; // Cov on z
        msg.pose.covariance[21] = 10000.0;
        msg.pose.covariance[28] = 10000.0;
        msg.pose.covariance[35] = 10000.0;
        range_pub_->publish(msg);
        (void)hardware_timestamp_usec;
    }
    
    std::atomic<float> roll_{0.0f}, pitch_{0.0f};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    LowPassFilter<float> vx_filter_{0.15f}; 
    LowPassFilter<float> vy_filter_{0.15f};
    LowPassFilter<float> range_filter_{0.3f};




private:
    int socket_fd_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr flow_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr range_pub_;
    
    std::thread rx_thread_;
    std::atomic<bool> is_running_;
    CanardInstance canard_;
    alignas(8) uint8_t canard_memory_pool_[16384]; 

    bool init_socketcan(const std::string& iface_name) {
        struct sockaddr_can addr; struct ifreq ifr;
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) return false;
        std::strncpy(ifr.ifr_name, iface_name.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) return false;
        addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
        int flags = fcntl(socket_fd_, F_GETFL, 0); fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
        int timestamp_on = 1;
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_TIMESTAMP, &timestamp_on, sizeof(timestamp_on)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to enable SO_TIMESTAMP");
        }

        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) return false;
        return true;
    }
    
    // listen frame of Hflow + extract timestamp + send all of this to libcanard
    void can_rx_loop() { 
        struct pollfd pfd;
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;

        struct can_frame frame;
        struct iovec iov;
        struct msghdr msg;
        char ctrlmsg[CMSG_SPACE(sizeof(struct timeval))];

        iov.iov_base = &frame;
        iov.iov_len = sizeof(frame);

        msg.msg_name = nullptr;
        msg.msg_namelen = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = ctrlmsg;
        msg.msg_controllen = sizeof(ctrlmsg);

        while (is_running_) {
            int ret = poll(&pfd, 1, 100);

            if (ret > 0 && (pfd.revents & POLLIN)) {
                while (recvmsg(socket_fd_, &msg, 0) > 0) {
                    
                    uint64_t timestamp_usec = 0;
                    struct cmsghdr *cmsg;
                    for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
                        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
                            struct timeval *tv = (struct timeval *)CMSG_DATA(cmsg);
                            timestamp_usec = static_cast<uint64_t>(tv->tv_sec) * 1000000ULL + tv->tv_usec;
                            break;
                        }
                    }
                    if (timestamp_usec == 0) {
                        timestamp_usec = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();
                    }

                    CanardCANFrame canard_frame;
                    canard_frame.id = frame.can_id;
                    canard_frame.data_len = frame.can_dlc;
                    std::memcpy(canard_frame.data, frame.data, frame.can_dlc);

                    canardHandleRxFrame(&canard_, &canard_frame, timestamp_usec);
                }
            }
        }
    }

};

//CALLBACKS LIBCANARD -> traduction of bytes in values
bool shouldAcceptTransfer(const CanardInstance* ins, uint64_t* out_data_type_signature, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id) {
    (void)ins; (void)transfer_type; (void)source_node_id;

    if (data_type_id == UAVCAN_EQUIPMENT_OPTICAL_FLOW_ID) {
        *out_data_type_signature = 0x6a908866bcb49c18ULL; // signatur Flow 
        return true;
    }
    if (data_type_id == UAVCAN_EQUIPMENT_RANGE_SENSOR_ID) {
        *out_data_type_signature = 0x68fffe70fc771952ULL; // signatur Telemeter
        return true;
    }
    return false;
}

void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer) {
    auto* node = static_cast<DronecanBridgeComponent*>(ins->user_reference);

     // OPTICAL FLOW
    if (transfer->data_type_id == UAVCAN_EQUIPMENT_OPTICAL_FLOW_ID && transfer->payload_len >= 21) {
        
        // declaration of the parameter
        uint32_t dt_bits = 0, gyro_x_bits = 0, gyro_y_bits = 0, flow_x_bits = 0, flow_y_bits = 0;
        uint8_t quality = 0;

        // decoding binary struct (21 octets)
        canardDecodeScalar(transfer, 0, 32, false, &dt_bits);      // integration_interval (Octets 0-3)
        canardDecodeScalar(transfer, 32, 32, false, &gyro_x_bits); // rate_gyro_integral_x (Octets 4-7)
        canardDecodeScalar(transfer, 64, 32, false, &gyro_y_bits); // rate_gyro_integral_y (Octets 8-11)
        canardDecodeScalar(transfer, 96, 32, false, &flow_x_bits); // flow_integral_x (Octets 12-15)
        canardDecodeScalar(transfer, 128, 32, false, &flow_y_bits);// flow_integral_y (Octets 16-19)
        canardDecodeScalar(transfer, 160, 8, false, &quality);     // quality (Octet 20)

        // conversion (bytes to floats)
        float dt_sec, gyro_x_rad, gyro_y_rad, flow_x_rad, flow_y_rad;
        std::memcpy(&dt_sec, &dt_bits, 4);
        std::memcpy(&gyro_x_rad, &gyro_x_bits, 4);
        std::memcpy(&gyro_y_rad, &gyro_y_bits, 4);
        std::memcpy(&flow_x_rad, &flow_x_bits, 4);
        std::memcpy(&flow_y_rad, &flow_y_bits, 4);
        
        //constexpr float QUALITY_MIN = 90.0f;   // caliber by change /optical_flow/quality on diverse textures max =255, From PX4 (LPE flow.cpp)
        constexpr float DT_MIN = 1e-6f, DT_MAX = 0.5f;
        constexpr float TILT_MAX = 0.5f;       // ~28.6°, comme PX4
        
        float roll = node->roll_.load();
        float pitch = node->pitch_.load();

        // calcul velocity with gyro moove
        constexpr float QUALITY_FLOOR = 40.0f;  // below this threshold, data unusable even when weighted
	if (quality >= QUALITY_FLOOR && dt_sec > DT_MIN && dt_sec < DT_MAX &&
	    node->current_distance_z_ > 0.05f &&
	    std::fabs(roll) < TILT_MAX && std::fabs(pitch) < TILT_MAX) {
	    
	    float flow_rate_x = (flow_x_rad - gyro_x_rad) / dt_sec;// rad/s
            float flow_rate_y = (flow_y_rad - gyro_y_rad) / dt_sec;
	    constexpr float FLOW_RATE_MAX = 7.2f; // pysical limit of PAA3905
	    
	    if (std::fabs(flow_rate_x) < FLOW_RATE_MAX && std::fabs(flow_rate_y) < FLOW_RATE_MAX) {
	    
	        float d = node->current_distance_z_; // * cosf(roll) * cosf(pitch);
		float raw_vel_x = -flow_rate_y * d;
                float raw_vel_y = flow_rate_x * d;

		float vel_x = node->vx_filter_.update(raw_vel_x);
                float vel_y = node->vy_filter_.update(raw_vel_y);
                
                // 4. Dynamic covariance modeling (Like PX4)
                // - If quality is perfect (255), the factor is 1.0.
                // - If altitude exceeds 1 meter, the reported noise is increased exponentially.
		float base_cov = 0.05f; 
                float quality_factor = 255.0f / static_cast<float>(quality);
                float dist_factor = std::max(1.0f, d * d); 
                float dynamic_cov = base_cov * quality_factor * dist_factor;

                node->publish_flow_data(vel_x, vel_y, dynamic_cov, transfer->timestamp_usec);
            }
        } else {
            // clear the filters if the sensor loses the ground.
            node->vx_filter_.reset();
            node->vy_filter_.reset();
        }
    }
    
    // LASER RANGE 
    else if (transfer->data_type_id == UAVCAN_EQUIPMENT_RANGE_SENSOR_ID && transfer->payload_len >= 15) {
        
        uint16_t range_bits = 0;
        
        // The ‘range’ field (float16) begins exactly at the 104th bit of the message
        canardDecodeScalar(transfer, 104, 16, false, &range_bits);
              
        
        // Converting CAN float16 to native C++ float32
        float distance_m = canardConvertFloat16ToNativeFloat(range_bits);
        
        // The distance is only published if it is valid (the sensor returns 0.0 if it is out of range)
	if (distance_m > 0.005f) {
	    node->publish_range_data(distance_m, transfer->timestamp_usec);
	}
    }
}
} 

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uavcan_bridge::DronecanBridgeComponent)
