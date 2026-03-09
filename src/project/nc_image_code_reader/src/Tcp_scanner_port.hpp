#pragma once
#include <boost/asio.hpp>
#include <memory>
#include <vector>
#include <functional>
#include <iostream>
#include "util/logger.hpp"

namespace NaviFra {

class TCPScannerPort
{
public:
    TCPScannerPort(boost::asio::io_context& io_context,
                   const std::string& host,
                   const std::string& port);

    void connect();
    void disconnect();
    void asyncWrite(const std::vector<uint8_t>& data, std::function<void()> onComplete);
    void asyncReadLine(std::function<void(const std::string)> onReceive);

    bool isOpen() const;

private:
    void doReadLine(std::function<void(const std::string)> onReceive);

private:
    boost::asio::ip::tcp::socket socket_;
    boost::asio::ip::tcp::resolver resolver_;
    std::string host_;
    std::string port_;
    boost::asio::streambuf streambuf_;
};

} // namespace NaviFra
