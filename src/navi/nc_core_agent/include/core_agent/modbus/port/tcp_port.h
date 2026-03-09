#ifndef NAVIFRA_MODBUS_TCP_PORT_H
#define NAVIFRA_MODBUS_TCP_PORT_H

#include <boost/asio.hpp>
#include <core_agent/modbus/modbus_constants.h>
#include <core_agent/modbus/modbus_exception.h>
#include <core_agent/modbus/port/modbus_port.h>
#include <core_agent/modbus/port/tcp_port.h>

#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace NaviFra {

class TCPPort : public ModbusPort {
public:
    TCPPort(boost::asio::io_context& io_context, const std::string& host, const std::string& port);

    void connect() override;

    void disconnect() override;

    void asyncWrite(const std::vector<uint8_t>& data, std::function<void()> onComplete) override;

    static uint16_t getNextTransactionID();

    virtual void sendFrame(const std::vector<uint8_t>& pdu, std::function<void()> onWriteComplete) override;

    void asyncRead(std::size_t size, std::function<void(const std::vector<uint8_t>&)> onReceive) override;

    bool isOpen() const override;

private:
    boost::asio::ip::tcp::socket socket_;
    boost::asio::ip::tcp::resolver resolver_;
    std::string host_;
    std::string port_;

    void printRawData(const std::string& label, const std::vector<uint8_t>& data);
};

}  // namespace NaviFra
#endif  // NAVIFRA_MODBUS_TCP_PORT_H
