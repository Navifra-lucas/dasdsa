#ifndef PLC_COMMUNICATOR_H
#define PLC_COMMUNICATOR_H

#include <string>
#include <array>
#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <ros/ros.h>

// POCO 라이브러리
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/NetException.h>
#include <Poco/Exception.h>
#include <Poco/Timespan.h>

namespace NaviFra {

class PLCCommunicator {
public:
    static constexpr size_t IO_DATA_SIZE = 64;  // 64바이트 고정
    
    struct ServerConfig {
        std::string bind_ip = "0.0.0.0";        // 바인드할 IP (모든 인터페이스)
        int port = 8080;                        // 리스닝 포트
        int max_connections = 1;                // 최대 연결 수 (보통 PLC 1개)
        int accept_timeout_ms = 1000;           // accept 타임아웃
        int send_timeout_ms = 2000;             // 송신 타임아웃
        int receive_timeout_ms = 2000;          // 수신 타임아웃
        bool keep_alive = true;                 // TCP Keep-Alive
        bool no_delay = true;                   // TCP_NODELAY
        int socket_buffer_size = 8192;          // 소켓 버퍼 크기
        bool reuse_address = true;              // SO_REUSEADDR
    };

    enum class ServerState {
        STOPPED,        // 서버 정지
        LISTENING,      // 연결 대기 중
        CONNECTED,      // PLC 연결됨
        ERROR           // 오류 상태
    };

public:
    PLCCommunicator();
    virtual ~PLCCommunicator();
    
    // 서버 관리
    bool startServer(const ServerConfig& config);
    bool startServer(const std::string& bind_ip, int port);
    bool stopServer();
    bool isServerRunning() const;
    bool isClientConnected() const;
    
    // 통신 (연결된 클라이언트와)
    bool sendAndReceive(const std::array<uint8_t, IO_DATA_SIZE>& send_data,
                       std::array<uint8_t, IO_DATA_SIZE>& recv_data);
 
    // 현재 출력 상태 관리
    void setOutputData(const std::array<uint8_t, IO_DATA_SIZE>& output_data);
    std::array<uint8_t, IO_DATA_SIZE> getOutputData() const;
    std::array<uint8_t, IO_DATA_SIZE> getLastInputData() const;
    
    // 상태 및 통계
    ServerState getServerState() const;
    std::string getServerStateString() const;
    std::string getConnectedClientIP() const;
    std::string getLastError() const;
    ServerConfig getConfig() const;
    size_t getSuccessfulTransactions() const;
    size_t getFailedTransactions() const;
    double getAverageTransactionTime() const;

private:
    // 서버 소켓 관리
    std::unique_ptr<Poco::Net::ServerSocket> server_socket_;
    std::unique_ptr<Poco::Net::StreamSocket> client_socket_;
    ServerConfig config_;
    std::atomic<ServerState> server_state_;
    
    // 스레드 관리
    std::thread accept_thread_;
    std::atomic<bool> should_stop_;
    
    // 클라이언트 정보
    std::string connected_client_ip_;
    mutable std::mutex client_mutex_;
    
    // 데이터 보호
    mutable std::mutex data_mutex_;
    std::array<uint8_t, IO_DATA_SIZE> current_output_;
    std::array<uint8_t, IO_DATA_SIZE> last_input_;
    
    // 통계 및 에러
    mutable std::mutex stats_mutex_;
    std::string last_error_;
    std::atomic<size_t> successful_transactions_;
    std::atomic<size_t> failed_transactions_;
    std::vector<double> transaction_times_;
    // 클래스 멤버 변수 추가
    std::chrono::steady_clock::time_point last_recv_log_time_;
    std::chrono::steady_clock::time_point last_send_log_time_;

    // 내부 메서드
    bool createServerSocket();
    void configureServerSocket();
    void acceptLoop();  // 백그라운드 스레드에서 실행
    bool acceptClient();
    void handleClientDisconnection();
    void configureClientSocket(Poco::Net::StreamSocket& socket);
    
    bool sendDataBlocking(const std::array<uint8_t, IO_DATA_SIZE>& data);
    bool receiveDataBlocking(std::array<uint8_t, IO_DATA_SIZE>& data);
    void updateStats(double transaction_time);
    void setLastError(const std::string& error);
    void setState(ServerState state);
};

} // namespace NaviFra

#endif // PLC_COMMUNICATOR_H