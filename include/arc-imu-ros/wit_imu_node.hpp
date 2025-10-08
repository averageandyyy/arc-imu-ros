#ifndef WIT_IMU_NODE_HPP
#define WIT_IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "arc-imu-ros/device/witHWT9073.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class WitImuNode : public rclcpp::Node
{
public:
    WitImuNode();
    ~WitImuNode();

private:
    void polling_loop();
    void reset_orientation_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    sensor_msgs::msg::Imu convert_to_ros2_msg(const WitHWT9073::IMUData &imu_data);
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_orientation_service_;
    std::unique_ptr<WitHWT9073> wit_imu_device_;

    std::string port_name_;
    int baud_rate_;
    std::string frame_id_;
    std::string imu_topic_;
    std::string reset_orientation_service_topic_;

    std::thread polling_thread_;
    std::atomic<bool> running_;
    std::mutex imu_data_mutex_;
};

#endif // WIT_IMU_NODE_HPP