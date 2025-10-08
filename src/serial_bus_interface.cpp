#include "arc-imu-ros/serial_bus_interface.hpp"
#include <exception>

SerialBusInterface::SerialBusInterface(const std::string &port_name, const int &baud_rate,
                                       boost::asio::serial_port_base::parity parity_option,
                                       boost::asio::serial_port_base::character_size char_size_option,
                                       boost::asio::serial_port_base::flow_control flow_control_option,
                                       boost::asio::serial_port_base::stop_bits stop_bits_option)
    : io_service_(), serial_port_(io_service_)
{
    open(port_name, baud_rate, parity_option, char_size_option, flow_control_option, stop_bits_option);
}

void SerialBusInterface::open(const std::string &port_name, const int &baud_rate,
                              boost::asio::serial_port_base::parity parity_option,
                              boost::asio::serial_port_base::character_size char_size_option,
                              boost::asio::serial_port_base::flow_control flow_control_option,
                              boost::asio::serial_port_base::stop_bits stop_bits_option)
{
    if (serial_port_.is_open())
    {
        serial_port_.close();
    }

    serial_port_.open(port_name);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_port_.set_option(parity_option);
    serial_port_.set_option(char_size_option);
    serial_port_.set_option(flow_control_option);
    serial_port_.set_option(stop_bits_option);

    std::cout << "Serial port " << port_name << " opened at baud rate " << baud_rate << std::endl;
}

SerialBusInterface::~SerialBusInterface()
{
    if (serial_port_.is_open())
    {
        serial_port_.close();
    }
}

size_t SerialBusInterface::read(uint8_t *buf, const size_t &size)
{
    size_t total_read = 0;

    while (total_read < size)
    {
        try
        {
            total_read += serial_port_.read_some(boost::asio::buffer(buf + total_read, size - total_read));
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Error reading from serial port: %s\n", e.what());
            return 0;
        }
    }

    return total_read;
}

void SerialBusInterface::write(const uint8_t *buf, const size_t &size)
{
    size_t total_written = 0;

    while (total_written < size)
    {
        try
        {
            total_written += serial_port_.write_some(boost::asio::buffer(buf + total_written, size - total_written));
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Error writing to serial port: %s\n", e.what());
            return;
        }
    }

    std::cout << "Wrote " << total_written << " bytes to serial port." << std::endl;
}

bool SerialBusInterface::is_open() const
{
    return serial_port_.is_open();
}