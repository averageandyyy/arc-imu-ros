#ifndef SERIAL_BUS_INTERFACE_HPP
#define SERIAL_BUS_INTERFACE_HPP

#include <string>
#include <cstddef>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio.hpp>
#include <iostream>

class SerialBusInterface
{
public:
    SerialBusInterface(const std::string &port_name, const int &baud_rate,
                       boost::asio::serial_port_base::parity parity_option = boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
                       boost::asio::serial_port_base::character_size char_size_option = boost::asio::serial_port_base::character_size(8),
                       boost::asio::serial_port_base::flow_control flow_control_option = boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none),
                       boost::asio::serial_port_base::stop_bits stop_bits_option = boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    ~SerialBusInterface();

    void open(const std::string &port_name, const int &baud_rate,
              boost::asio::serial_port_base::parity parity_option = boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
              boost::asio::serial_port_base::character_size char_size_option = boost::asio::serial_port_base::character_size(8),
              boost::asio::serial_port_base::flow_control flow_control_option = boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none),
              boost::asio::serial_port_base::stop_bits stop_bits_option = boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    size_t read(uint8_t *buf, const size_t &size);
    void write(const uint8_t *buf, const size_t &size);
    bool is_open() const;

private:
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;
};

#endif // SERIAL_BUS_INTERFACE_HPP