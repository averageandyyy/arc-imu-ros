#ifndef WITHT9073_HPP
#define WITHT9073_HPP

#include "arc-imu-ros/device/device.hpp"

class WitHWT9073 : public Device
{
    /*
    At this point, the data comes in as Roll (10), 6 bytes of junk, Yaw (10), 6 bytes of junk, Angular Velocity (10), 6 bytes of junk,
    Pitch (10), 6 bytes of junk, Acceleration (10), 6 bytes of junk
    Total of 80 bytes for a full data set.
    */
public:
    struct IMUData
    {
        float acceleration[3];       // Acceleration in m/s² (ax, ay, az)
        float angular_velocity[3];   // Angular velocity in rad/s (wx, wy, wz)
        float orientation[3];        // Orientation in radians (roll, pitch, yaw)
        float orientation_offset[3]; // Offset for orientation (roll_offset, pitch_offset, yaw_offset)
    };

    WitHWT9073(const std::string &port_name, const int &baud_rate, std::mutex &data_mutex);

    void initialize() override;

    bool get_data(uint8_t *data) override;

    void reset() override;

    static constexpr size_t FRAME_SIZE = 10;                                                                        // 1 (header) + 1 (type) + 8 (data)
    static constexpr size_t HEADER_SIZE = 1;                                                                        // 1 byte for header
    static constexpr size_t DATA_HEADER_SIZE = 1;                                                                   // 1 byte for data type
    static constexpr size_t ORIENTATION_HEADER_SIZE = 1;                                                            // 1 byte for orientation data type
    static constexpr size_t FULL_FRAME_SIZE = 80;                                                                   // Full data frame size, as per above description
    static constexpr size_t JUNK_SIZE = 6;                                                                          // 6 bytes of junk data between actual data
    static constexpr size_t DATA_PACKET_SIZE = 8;                                                                   // 8 bytes of actual data in each packet
    static constexpr size_t DATA_SIZE = FULL_FRAME_SIZE - HEADER_SIZE - DATA_HEADER_SIZE - ORIENTATION_HEADER_SIZE; // Size of actual data in full frame
    static constexpr float ACCELERATION_SCALE = 16.0f;                                                              // ±16g
    static constexpr float ANGULAR_VELOCITY_SCALE = 2000.0f;                                                        // ±2000°/s
    static constexpr float GRAVITY = 9.80665f;                                                                      // m/s²
    static constexpr float MAX_SENSOR_VALUE = 32768.0f;                                                             // 2^15
    static constexpr float ANGLE_SCALE = 1000.0f;                                                                   // Scale for roll, pitch, yaw in degrees
    static constexpr float DEG2RAD = 3.14159265359f / 180.0f;                                                       // Degrees to radians conversion factor
    static constexpr uint8_t UNLOCK[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};                                             // Unlock command
    // static constexpr uint8_t UNLOCK[] = {0xB5, 0x88, 0x69, 0xAA, 0xFF}; // Unlock command
    static constexpr uint8_t INIT_ITEMS[] = {0xFF, 0xAA, 0x02, 0x0E, 0x00}; // Initialization command
    // static constexpr uint8_t INIT_ITEMS[] = {0x00, 0x02, 0x02, 0xAA, 0xFF};   // Initialization command
    static constexpr uint8_t BAUD_RATE_200K[] = {0xFF, 0xAA, 0x04, 0x05, 0x00};   // Set baud rate to 200000
    static constexpr uint8_t OUTPUT_RATE_50HZ[] = {0xFF, 0xAA, 0x03, 0x08, 0x00}; // Set data rate to 50Hz
    static constexpr uint8_t SAVE_ITEMS[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};       // Save command
    // static constexpr uint8_t SAVE_ITEMS[] = {0x00, 0x00, 0x00, 0xAA, 0xFF}; // Save command
    static constexpr uint8_t REBOOT[] = {0xFF, 0xAA, 0x00, 0xFF, 0x00}; // Reboot command

    IMUData get_imu_data() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return imu_data_;
    }

private:
    enum class FrameHeader : uint8_t
    {
        START_BYTE = 0x55,
        ACCELERATION = 0x51,
        ANGULAR_VELOCITY = 0x52,
        ORIENTATION = 0x53,
        MAGNETIC_FIELD = 0x54
    };

    enum class AngleAxis : uint8_t
    {
        ROLL = 0x01,
        PITCH = 0x02,
        YAW = 0x03
    };

    IMUData imu_data_;

    // Data validation methods
    bool is_valid_data_header(const uint8_t &data_header);
    bool is_acceleration_frame(const uint8_t &data_header);
    bool is_angular_velocity_frame(const uint8_t &data_header);
    bool is_orientation_frame(const uint8_t &data_header);
    bool has_valid_data_headers(const uint8_t *data);
    bool is_valid_start_byte(const uint8_t &start_byte);
    bool is_roll_orientation_frame(const uint8_t &data_header);
    bool is_pitch_orientation_frame(const uint8_t &data_header);
    bool is_yaw_orientation_frame(const uint8_t &data_header);

    // Data parsing methods
    void parse_acceleration_data(const uint8_t *data);
    void parse_angular_velocity_data(const uint8_t *data);
    void parse_orientation_data(const uint8_t *data);
};

#endif // WITHT9073_HPP