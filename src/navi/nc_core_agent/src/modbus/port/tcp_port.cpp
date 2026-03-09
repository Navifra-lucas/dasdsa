
#include "core_agent/core_agent.h"

#include <core_agent/modbus/port/tcp_port.h>

using namespace NaviFra;

TCPPort::TCPPort(boost::asio::io_context& io_context, const std::string& host, const std::string& port)
    : socket_(io_context)
    , resolver_(io_context)
    , host_(host)
    , port_(port)
{
}

void TCPPort::connect()
{
    auto endpoints = resolver_.resolve(host_, port_);
    boost::asio::connect(socket_, endpoints);
}

void TCPPort::disconnect()
{
    socket_.close();
}

void TCPPort::asyncWrite(const std::vector<uint8_t>& data, std::function<void()> onComplete)
{
    boost::asio::async_write(socket_, boost::asio::buffer(data), [onComplete](boost::system::error_code, std::size_t) {
        if (onComplete)
            onComplete();
    });
}

uint16_t TCPPort::getNextTransactionID()
{
    static uint16_t transactionID = 1;  // 초기값 설정
    uint16_t currentID = transactionID;

    // 최대 값에 도달하면 1로 순환
    transactionID = (transactionID == 0xFFFF) ? 1 : transactionID + 1;

    return currentID;
}

// TCPPort에서 TCP 헤더를 추가한 후 전송
void TCPPort::sendFrame(const std::vector<uint8_t>& pdu, std::function<void()> onWriteComplete)
{
    std::vector<uint8_t> packet;

    // TCP 헤더 작성
    uint16_t transaction_id = getNextTransactionID();
    uint16_t length = pdu.size();  // PDU 길이 (슬레이브 주소 포함)
    packet.push_back((transaction_id >> 8) & 0xFF);
    packet.push_back(transaction_id & 0xFF);
    packet.push_back(0x00);  // Protocol ID (Modbus에서는 항상 0)
    packet.push_back(0x00);
    packet.push_back((length >> 8) & 0xFF);
    packet.push_back(length & 0xFF);

    // PDU 추가
    packet.insert(packet.end(), pdu.begin(), pdu.end());

    // printRawData("Sending Frame", packet);

    socket_.async_send(boost::asio::buffer(packet), [onWriteComplete](boost::system::error_code, std::size_t) { onWriteComplete(); });
}

void TCPPort::asyncRead(std::size_t size, std::function<void(const std::vector<uint8_t>&)> onReceive)
{
    auto buffer = std::make_shared<std::vector<uint8_t>>(size);
    auto header_buffer = std::make_shared<std::vector<uint8_t>>(MBAP_HEADER_SIZE);

    // 1단계: MBAP 헤더 수신
    boost::asio::async_read(
        socket_, boost::asio::buffer(*header_buffer),
        [this, buffer, header_buffer, onReceive](boost::system::error_code ec, std::size_t bytes_transferred) {
            if (!ec && bytes_transferred == MBAP_HEADER_SIZE) {
                // MBAP 헤더 파싱
                uint16_t transactionID = (header_buffer->at(0) << 8) | header_buffer->at(1);
                uint16_t protocolID = (header_buffer->at(2) << 8) | header_buffer->at(3);
                uint16_t length = (header_buffer->at(4) << 8) | header_buffer->at(5);

                if (protocolID != 0) {
                    throw std::runtime_error("Invalid protocol ID: " + std::to_string(protocolID));
                }

                // 전체 데이터 길이 계산 (MBAP 헤더 제외)
                std::size_t remaining_length = length - 1;  // Unit ID 제외

                // 2단계: 나머지 데이터 수신
                buffer->resize(MBAP_HEADER_SIZE + remaining_length);
                std::copy(header_buffer->begin(), header_buffer->end(), buffer->begin());  // 헤더 복사

                boost::asio::async_read(
                    socket_, boost::asio::buffer(buffer->data() + MBAP_HEADER_SIZE, remaining_length),
                    [buffer, onReceive](boost::system::error_code ec, std::size_t bytes_transferred) {
                        if (!ec) {
                            // 에러 응답 확인
                            uint8_t functionCode = buffer->at(MBAP_HEADER_SIZE);
                            if (functionCode & 0x80) {  // 에러 응답 확인 (functionCode의 상위 비트가 1로 설정됨)
                                uint8_t errorCode = buffer->at(MBAP_HEADER_SIZE + 1);
                                std::string errorMessage;

                                switch (errorCode) {
                                    case 0x01:
                                        errorMessage = "Illegal Function";
                                        break;
                                    case 0x02:
                                        errorMessage = "Illegal Data Address";
                                        break;
                                    case 0x03:
                                        errorMessage = "Illegal Data Value";
                                        break;
                                    case 0x04:
                                        errorMessage = "Slave Device Failure";
                                        break;
                                    default:
                                        errorMessage = "Unknown Error Code";
                                        break;
                                }

                                throw std::runtime_error("Modbus Error: " + errorMessage);
                            }

                            // 정상 응답 처리
                            if (onReceive) {
                                std::vector<uint8_t> pdu(
                                    buffer->begin() + MBAP_HEADER_SIZE, buffer->begin() + MBAP_HEADER_SIZE + bytes_transferred);
                                onReceive(pdu);  // 헤더를 제외한 PDU 데이터만 콜백 호출
                            }
                        }
                        else {
                            throw std::runtime_error("Error during async read: " + ec.message());
                        }
                    });
            }
            else {
                throw std::runtime_error("Error reading MBAP header: " + ec.message());
            }
        });
}

bool TCPPort::isOpen() const
{
    return socket_.is_open();
}

void TCPPort::printRawData(const std::string& label, const std::vector<uint8_t>& data)
{
    std::ostringstream oss;
    oss << label << " [";
    for (uint8_t byte : data) {
        oss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    oss << "]";
    NLOG(info) << oss.str();
}