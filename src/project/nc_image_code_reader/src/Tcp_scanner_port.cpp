#include "Tcp_scanner_port.hpp"

using namespace NaviFra;
using boost::asio::ip::tcp;

TCPScannerPort::TCPScannerPort(boost::asio::io_context& io_context,
                               const std::string& host,
                               const std::string& port)
    : socket_(io_context)
    , resolver_(io_context)
    , host_(host)
    , port_(port)
{
}

void TCPScannerPort::connect()
{
    auto endpoints = resolver_.resolve(host_, port_);
    boost::asio::connect(socket_, endpoints);
}

void TCPScannerPort::disconnect()
{
    boost::system::error_code ec;
    socket_.cancel(ec);  // pending async read cancel
    socket_.close(ec);
}

void TCPScannerPort::asyncWrite(const std::vector<uint8_t>& data,
                                std::function<void()> onComplete)
{
    boost::asio::async_write(socket_, boost::asio::buffer(data),
        [onComplete](boost::system::error_code, std::size_t) {
            if (onComplete)
                onComplete();
        });
}

void TCPScannerPort::asyncReadLine(std::function<void(const std::string)> onReceive)
{
    doReadLine(std::move(onReceive));
}

void TCPScannerPort::doReadLine(std::function<void(const std::string)> onReceive)
{
    boost::asio::async_read_until(socket_, streambuf_, '\x03',
        [this, onReceive = std::move(onReceive)](boost::system::error_code ec, std::size_t bytes_transferred) mutable {
            if (!ec)
            {
                std::istream is(&streambuf_);
                std::string line;
                std::getline(is, line, '\x03');  // ETX 기준 읽기

                if (!line.empty() && line.back() == '\r')
                    line.pop_back();

                // STX 제거
                if (!line.empty() && line.front() == 0x02)
                    line.erase(line.begin());

                if (onReceive)
                    onReceive(line);

                // 재귀 호출로 계속 읽기
                doReadLine(std::move(onReceive));
            }
            else
            {
                NLOG(error) << "[Scanner] Read error: " << ec.message();
            }
        });
}

bool TCPScannerPort::isOpen() const
{
    return socket_.is_open();
}
