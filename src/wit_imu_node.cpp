#include "arc-imu-ros/wit_imu_node.hpp"

WitImuNode::WitImuNode()
    : Node("wit_imu_node"), running_(true)
{
    this->declare_parameter("port_name", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 2000000);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("imu_topic", "imu/data");
    this->declare_parameter("reset_orientation_service_topic", "imu/reset_orientation");

    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    reset_orientation_service_topic_ = this->get_parameter("reset_orientation_service_topic").as_string();

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    reset_orientation_service_ = this->create_service<std_srvs::srv::Trigger>(
        reset_orientation_service_topic_,
        std::bind(&WitImuNode::reset_orientation_callback, this, std::placeholders::_1, std::placeholders::_2));

    wit_imu_device_ = std::make_unique<WitHWT9073>(port_name_, baud_rate_, imu_data_mutex_);

    RCLCPP_INFO(this->get_logger(), "WitImuNode initialized on port %s at baud rate %d", port_name_.c_str(), baud_rate_);

    polling_thread_ = std::thread(&WitImuNode::polling_loop, this);
}

WitImuNode::~WitImuNode()
{
    running_ = false;
    if (polling_thread_.joinable())
    {
        polling_thread_.join();
    }
}

void WitImuNode::polling_loop()
{
    while (running_ && rclcpp::ok())
    {
        uint8_t buffer[WitHWT9073::FRAME_SIZE]{};
        if (wit_imu_device_->get_data(buffer))
        {
            auto imu_data = wit_imu_device_->get_imu_data();

            if (!imu_data_initialized_)
            {
                RCLCPP_INFO(this->get_logger(), "Initial IMU data received.");
                wit_imu_device_->reset();
                imu_data_initialized_ = true;
            }

            auto imu_msg = convert_to_ros2_msg(imu_data);
            imu_publisher_->publish(imu_msg);
        }
    }
}

sensor_msgs::msg::Imu WitImuNode::convert_to_ros2_msg(const WitHWT9073::IMUData &imu_data)
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = frame_id_;

    // Fill linear acceleration
    imu_msg.linear_acceleration.x = imu_data.acceleration[0];
    imu_msg.linear_acceleration.y = imu_data.acceleration[1];
    imu_msg.linear_acceleration.z = imu_data.acceleration[2];

    // Fill angular velocity
    imu_msg.angular_velocity.x = imu_data.angular_velocity[0];
    imu_msg.angular_velocity.y = imu_data.angular_velocity[1];
    imu_msg.angular_velocity.z = imu_data.angular_velocity[2];

    // Convert roll, pitch, yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(imu_data.orientation[0] - imu_data.orientation_offset[0],
             imu_data.orientation[1] - imu_data.orientation_offset[1],
             imu_data.orientation[2] - imu_data.orientation_offset[2]);
    imu_msg.orientation = tf2::toMsg(q);

    // Print RPY
    RCLCPP_INFO(this->get_logger(), "RPY: [%.3f, %.3f, %.3f]", imu_data.orientation[0] - imu_data.orientation_offset[0], imu_data.orientation[1] - imu_data.orientation_offset[1], imu_data.orientation[2] - imu_data.orientation_offset[2]);

    // Set covariance (example values, adjust as needed)
    for (int i = 0; i < 9; ++i)
    {
        imu_msg.orientation_covariance[i] = 0.01;         // Example covariance for orientation
        imu_msg.angular_velocity_covariance[i] = 0.01;    // Example covariance for angular velocity
        imu_msg.linear_acceleration_covariance[i] = 0.04; // Example covariance for linear acceleration
    }

    return imu_msg;
}

void WitImuNode::reset_orientation_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Unused parameter
    wit_imu_device_->reset();
    response->success = true;
    response->message = "IMU orientation reset successfully.";
    RCLCPP_INFO(this->get_logger(), "IMU orientation reset.");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WitImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
