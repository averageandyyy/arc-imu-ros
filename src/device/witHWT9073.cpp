#include "arc-imu-ros/device/witHWT9073.hpp"

WitHWT9073::WitHWT9073(const std::string &port_name, const int &baud_rate, std::mutex &data_mutex)
    : Device(port_name, baud_rate, data_mutex), imu_data_()
{
    initialize();
}

void WitHWT9073::initialize()
{
    if (!serial_bus_interface_->is_open())
    {
        throw std::runtime_error("Failed to open serial port for WitHWT9073 initialization.");
    }

    serial_bus_interface_->write(UNLOCK, sizeof(UNLOCK));
    sleep(1);
    serial_bus_interface_->write(OUTPUT_RATE_50HZ, sizeof(OUTPUT_RATE_50HZ));
    sleep(1);
    serial_bus_interface_->write(INIT_ITEMS, sizeof(INIT_ITEMS));
    sleep(1);
    serial_bus_interface_->write(SAVE_ITEMS, sizeof(SAVE_ITEMS));
    sleep(1);
    serial_bus_interface_->write(REBOOT, sizeof(REBOOT));
    sleep(1);
    std::cout << "WitHWT9073 initialized successfully." << std::endl;
}

bool WitHWT9073::is_acceleration_frame(const uint8_t &data_header)
{
    return data_header == static_cast<uint8_t>(FrameHeader::ACCELERATION);
}

bool WitHWT9073::is_angular_velocity_frame(const uint8_t &data_header)
{
    return data_header == static_cast<uint8_t>(FrameHeader::ANGULAR_VELOCITY);
}

bool WitHWT9073::is_orientation_frame(const uint8_t &data_header)
{
    return data_header == static_cast<uint8_t>(FrameHeader::ORIENTATION);
}

bool WitHWT9073::is_valid_data_header(const uint8_t &data_header)
{
    return is_acceleration_frame(data_header) ||
           is_angular_velocity_frame(data_header) ||
           is_orientation_frame(data_header);
}

void WitHWT9073::parse_acceleration_data(const uint8_t *data)
{
    for (size_t i = 0; i < 3; i++)
    {
        int16_t raw_value = static_cast<int16_t>((data[2 * i + 1] << 8) | data[2 * i]);
        imu_data_.acceleration[i] = (static_cast<float>(raw_value) / MAX_SENSOR_VALUE) * ACCELERATION_SCALE * GRAVITY;
    }

    std::cout << "Received Acceleration Data." << std::endl;
}

void WitHWT9073::parse_angular_velocity_data(const uint8_t *data)
{
    for (size_t i = 0; i < 3; i++)
    {
        int16_t raw_value = static_cast<int16_t>((data[2 * i + 1] << 8) | data[2 * i]);
        imu_data_.angular_velocity[i] = (static_cast<float>(raw_value) / MAX_SENSOR_VALUE) * ANGULAR_VELOCITY_SCALE;
    }
    std::cout << "Received Angular Velocity Data." << std::endl;
}

void WitHWT9073::parse_orientation_data(const uint8_t *data)
{
    if (data[0] == static_cast<uint8_t>(AngleAxis::ROLL))
    {
        int32_t raw_value = (static_cast<int32_t>(data[5] << 24) | static_cast<int32_t>(data[4] << 16) |
                             static_cast<int32_t>(data[3] << 8) | static_cast<int32_t>(data[2]));
        imu_data_.orientation[0] = (static_cast<float>(raw_value) / ANGLE_SCALE) * DEG2RAD;
        std::cout << "Received Roll Data." << std::endl;
    }
    else if (data[0] == static_cast<uint8_t>(AngleAxis::PITCH))
    {
        int32_t raw_value = (static_cast<int32_t>(data[5] << 24) | static_cast<int32_t>(data[4] << 16) |
                             static_cast<int32_t>(data[3] << 8) | static_cast<int32_t>(data[2]));
        imu_data_.orientation[1] = (static_cast<float>(raw_value) / ANGLE_SCALE) * DEG2RAD;
        std::cout << "Received Pitch Data." << std::endl;
    }
    else if (data[0] == static_cast<uint8_t>(AngleAxis::YAW))
    {
        int32_t raw_value = (static_cast<int32_t>(data[5] << 24) | static_cast<int32_t>(data[4] << 16) |
                             static_cast<int32_t>(data[3] << 8) | static_cast<int32_t>(data[2]));
        imu_data_.orientation[2] = (static_cast<float>(raw_value) / ANGLE_SCALE) * DEG2RAD;
        std::cout << "Received Yaw Data." << std::endl;
    }
    else
    {
        std::cerr << "Unknown orientation axis." << std::endl;
    }
}

bool WitHWT9073::get_data(uint8_t *data)
{
    if (!serial_bus_interface_->is_open())
    {
        std::cerr << "Serial port not open. Cannot read data." << std::endl;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        size_t bytes_read = serial_bus_interface_->read(data, HEADER_SIZE);

        if (bytes_read != HEADER_SIZE || data[0] != static_cast<uint8_t>(FrameHeader::START_BYTE))
        {
            std::cerr << "Invalid frame header." << std::endl;
            std::cerr << "Received byte: " << std::hex << static_cast<int>(data[0]) << std::dec << std::endl;
            return false;
        }

        bytes_read += serial_bus_interface_->read(data + HEADER_SIZE, DATA_HEADER_SIZE);
        if (bytes_read != HEADER_SIZE + DATA_HEADER_SIZE || !is_valid_data_header(data[1]))
        {
            std::cerr << "Invalid data header." << std::endl;
            return false;
        }

        // Read the rest of the data frame byte by byte
        for (size_t i = 0; i < DATA_SIZE; ++i)
        {
            bytes_read += serial_bus_interface_->read(data + HEADER_SIZE + DATA_HEADER_SIZE + i, 1);
            if (bytes_read != HEADER_SIZE + DATA_HEADER_SIZE + i + 1)
            {
                std::cerr << "Incomplete data frame." << std::endl;
                return false;
            }
        }

        std::cout << "Data frame received successfully." << std::endl;

        if (is_acceleration_frame(data[1]))
        {
            parse_acceleration_data(data + HEADER_SIZE + DATA_HEADER_SIZE);
            return true;
        }
        else if (is_angular_velocity_frame(data[1]))
        {
            parse_angular_velocity_data(data + HEADER_SIZE + DATA_HEADER_SIZE);
            return true;
        }
        else if (is_orientation_frame(data[1]))
        {
            parse_orientation_data(data + HEADER_SIZE + DATA_HEADER_SIZE);
            return true;
        }

        std::cerr << "Unknown data type." << std::endl;
        return false;
    }
}

void WitHWT9073::reset()
{
    this->imu_data_.orientation_offset[0] = this->imu_data_.orientation[0];
    this->imu_data_.orientation_offset[1] = this->imu_data_.orientation[1];
    this->imu_data_.orientation_offset[2] = this->imu_data_.orientation[2];
    std::cout << "Orientation offsets reset." << std::endl;
}