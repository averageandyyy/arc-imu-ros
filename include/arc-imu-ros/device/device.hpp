#ifndef DEVICE_HPP
#define DEVICE_HPP

#include "arc-imu-ros/serial_bus_interface.hpp"
#include <memory>
#include <mutex>

class Device
{
    /* This class is a Device interface class that can be extended for future sensors.
       Has some pure virtual functions to be implemented by derived classes.
    */
public:
    Device(const std::string &port_name, const int &baud_rate, std::mutex &data_mutex)
        : serial_bus_interface_(std::make_unique<SerialBusInterface>(port_name, baud_rate)), data_mutex_(data_mutex) {}

    virtual ~Device() = default;

    virtual void initialize() = 0;            // Pure virtual function for initialization
    virtual bool get_data(uint8_t *data) = 0; // Pure virtual function to get data from the device/sensor
    virtual void reset() = 0;                 // Pure virtual function to reset the device/sensor

protected:
    std::unique_ptr<SerialBusInterface> serial_bus_interface_;
    std::mutex &data_mutex_;
};

#endif // DEVICE_HPP