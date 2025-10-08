#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "arc-imu-ros/serial_can_interface.hpp"
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ImuNode : public rclcpp::Node
{
public:
    ImuNode();

private:
    void timer_callback();
    void reset_orientation_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_orientation_service_;
    std::unique_ptr<SerialCanInterface> serial_can_interface_;

    std::string port_name_;
    int baud_rate_;
    std::string frame_id_;
    std::string imu_topic_;
    double publish_rate_;
    std::string reset_orientation_service_topic_;
    sensor_msgs::msg::Imu imu_msg_;
    std::vector<double> initial_orientation_{0.0, 0.0, 0.0};
    std::vector<double> orientation_offset_{0.0, 0.0, 0.0};
};

#endif // IMU_NODE_HPP