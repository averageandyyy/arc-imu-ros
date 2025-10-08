#include "arc-imu-ros/imu_node.hpp"

ImuNode::ImuNode()
    : Node("imu_node")
{
    this->declare_parameter("port_name", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 2000000);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("imu_topic", "imu/data");
    this->declare_parameter("publish_rate", 5.0);
    this->declare_parameter("reset_orientation_service_topic", "imu/reset_orientation");

    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    reset_orientation_service_topic_ = this->get_parameter("reset_orientation_service_topic").as_string();

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                                     std::bind(&ImuNode::timer_callback, this));
    reset_orientation_service_ = this->create_service<std_srvs::srv::Trigger>(
        reset_orientation_service_topic_,
        std::bind(&ImuNode::reset_orientation_callback, this, std::placeholders::_1, std::placeholders::_2));
    serial_can_interface_ = std::make_unique<SerialCanInterface>(port_name_, baud_rate_);

    imu_msg_ = sensor_msgs::msg::Imu();
    imu_msg_.header.frame_id = frame_id_;
}

void ImuNode::timer_callback()
{
    uint8_t buffer[13]{};
    size_t header_size = 1;
    size_t data_size = 7;

    // Read and validate header
    if (serial_can_interface_->read(buffer, header_size) != header_size || buffer[0] != 0x55)
    {
        return;
    }

    // Read CAN frame data
    if (serial_can_interface_->read(buffer + header_size, data_size) != data_size)
    {
        return;
    }

    // Process the received CAN frame data
    uint8_t data_1_L{};
    uint8_t data_1_H{};
    uint8_t data_2_L{};
    uint8_t data_2_H{};
    uint8_t data_3_L{};
    uint8_t data_3_H{};

    if (buffer[1] == 0x51 || buffer[1] == 0x52 || buffer[1] == 0x53)
    {
        data_1_L = buffer[2];
        data_1_H = buffer[3];
        data_2_L = buffer[4];
        data_2_H = buffer[5];
        data_3_L = buffer[6];
        data_3_H = buffer[7];
    }
    else
    {
        return; // Unknown data type
    }

    if (buffer[1] == 0x51) // Acceleration
    {
        imu_msg_.linear_acceleration.x = static_cast<int32_t>((data_1_H << 8) | data_1_L) / 32768.0 * 16.0 * 9.80665;
        imu_msg_.linear_acceleration.y = static_cast<int32_t>((data_2_H << 8) | data_2_L) / 32768.0 * 16.0 * 9.80665;
        imu_msg_.linear_acceleration.z = static_cast<int32_t>((data_3_H << 8) | data_3_L) / 32768.0 * 16.0 * 9.80665;
    }
    else if (buffer[1] == 0x52) // Angular Velocity
    {
        imu_msg_.angular_velocity.x = static_cast<int32_t>((data_1_H << 8) | data_1_L) / 32768.0 * 2000.0 * (M_PI / 180.0);
        imu_msg_.angular_velocity.y = static_cast<int32_t>((data_2_H << 8) | data_2_L) / 32768.0 * 2000.0 * (M_PI / 180.0);
        imu_msg_.angular_velocity.z = static_cast<int32_t>((data_3_H << 8) | data_3_L) / 32768.0 * 2000.0 * (M_PI / 180.0);
    }
    else if (buffer[1] == 0x53) // Orientation
    {
        if (data_1_L == 0x01) // Roll
        {
            initial_orientation_[0] = static_cast<double>((static_cast<int32_t>(data_3_H << 24)) | (static_cast<int32_t>(data_3_L << 16)) | (static_cast<int32_t>(data_2_H << 8)) | static_cast<int32_t>(data_2_L)) / 1000.0;
            initial_orientation_[0] -= orientation_offset_[0];
        }
        else if (data_1_L == 0x02) // Pitch
        {
            initial_orientation_[1] = static_cast<double>((static_cast<int32_t>(data_3_H << 24)) | (static_cast<int32_t>(data_3_L << 16)) | (static_cast<int32_t>(data_2_H << 8)) | static_cast<int32_t>(data_2_L)) / 1000.0;
            initial_orientation_[1] -= orientation_offset_[1];
        }
        else if (data_1_L == 0x03) // Yaw
        {
            initial_orientation_[2] = static_cast<double>((static_cast<int32_t>(data_3_H << 24)) | (static_cast<int32_t>(data_3_L << 16)) | (static_cast<int32_t>(data_2_H << 8)) | static_cast<int32_t>(data_2_L)) / 1000.0;
            initial_orientation_[2] -= orientation_offset_[2];
        }

        // Convert to quaternion
        tf2::Quaternion q;
        q.setRPY(initial_orientation_[0], initial_orientation_[1], initial_orientation_[2]);
        imu_msg_.orientation = tf2::toMsg(q);
    }

    imu_msg_.header.stamp = this->now();
    imu_publisher_->publish(imu_msg_);
}

void ImuNode::reset_orientation_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Unused parameter

    orientation_offset_[0] = initial_orientation_[0];
    orientation_offset_[1] = initial_orientation_[1];
    orientation_offset_[2] = initial_orientation_[2];

    response->success = true;
    response->message = "Orientation reset successfully.";
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}