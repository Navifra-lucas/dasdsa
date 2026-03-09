#ifndef NAVIFRA_MODBUS_RTU_PORT_H
#define NAVIFRA_MODBUS_RTU_PORT_H

#include <boost/asio.hpp>
#include <core_agent/modbus/modbus_exception.h>
#include <core_agent/modbus/port/modbus_port.h>
#include <core_agent/modbus/port/rtu_port.h>

#include <functional>
#include <iostream>
#include <vector>

namespace NaviFra {

class RTUPort : public ModbusPort {
public:
    RTUPort(boost::asio::io_context& io_context, const std::string& device, unsigned int baud_rate);

    void connect() override;

    void disconnect() override;

    bool isOpen() const override;

    void sendFrame(const std::vector<uint8_t>& pdu, std::function<void()> onWriteComplete) override;

    void asyncRead(std::size_t length, std::function<void(const std::vector<uint8_t>&)> onReceive) override;

    void asyncWrite(const std::vector<uint8_t>& data, std::function<void()> on_write) override;

private:
    std::size_t calculateExpectedLengthFromPartial(const std::vector<uint8_t>& data);

private:
    boost::asio::io_context& io_context_;
    boost::asio::serial_port serial_port_;
    std::string device_;
    unsigned int baud_rate_;
};
}  // namespace NaviFra

#endif  // NAVIFRA_MODBUS_RTU_PORT_H
