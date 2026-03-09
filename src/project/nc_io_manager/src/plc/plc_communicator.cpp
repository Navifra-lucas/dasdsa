#include "nc_io_manager/plc/plc_communicator.h"



#include "util/logger.hpp"

#include <Poco/Format.h>
#include <Poco/Logger.h>
namespace NaviFra {

PLCCommunicator::PLCCommunicator()
    : server_socket_(nullptr)
    , client_socket_(nullptr)
    , server_state_(ServerState::STOPPED)
    , should_stop_(false)
    , successful_transactions_(0)
    , failed_transactions_(0)
{
    // 데이터 초기화
    current_output_.fill(0);
    last_input_.fill(0);
    last_recv_log_time_ = std::chrono::steady_clock::now();
    last_send_log_time_ = std::chrono::steady_clock::now();

    LOG_INFO("PLC Server Communicator created");
}

PLCCommunicator::~PLCCommunicator()
{
    stopServer();
    LOG_INFO("PLC Server Communicator destroyed");
}

bool PLCCommunicator::startServer(const ServerConfig& config)
{
    if (server_state_ != ServerState::STOPPED) {
        LOG_WARNING("Server is already running");
        return true;
    }

    config_ = config;

    LOG_INFO("Starting PLC server on %s:%d", config_.bind_ip.c_str(), config_.port);

    try {
        if (!createServerSocket()) {
            return false;
        }

        setState(ServerState::LISTENING);
        should_stop_ = false;

        // Accept 스레드 시작
        accept_thread_ = std::thread(&PLCCommunicator::acceptLoop, this);

        LOG_INFO("PLC server started successfully, waiting for PLC connection...");
        return true;
    }
    catch (const Poco::Exception& ex) {
        setLastError("Failed to start server: " + ex.displayText());
        setState(ServerState::ERROR);
        return false;
    }
}

bool PLCCommunicator::startServer(const std::string& bind_ip, int port)
{
    ServerConfig config;
    config.bind_ip = bind_ip;
    config.port = port;
    return startServer(config);
}

bool PLCCommunicator::stopServer()
{
    if (server_state_ == ServerState::STOPPED) {
        return true;
    }

    LOG_INFO("Stopping PLC server...");

    // 정지 플래그 설정
    should_stop_ = true;

    // 클라이언트 연결 해제
    {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_socket_) {
            try {
                client_socket_->close();
            }
            catch (...) {
            }
            client_socket_.reset();
        }
    }

    // 서버 소켓 해제
    if (server_socket_) {
        try {
            server_socket_->close();
        }
        catch (...) {
        }
        server_socket_.reset();
    }

    // Accept 스레드 종료 대기
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }

    setState(ServerState::STOPPED);
    connected_client_ip_.clear();

    LOG_INFO("PLC server stopped");
    return true;
}

bool PLCCommunicator::isServerRunning() const
{
    return server_state_ != ServerState::STOPPED;
}

bool PLCCommunicator::isClientConnected() const
{
    std::lock_guard<std::mutex> lock(client_mutex_);
    return client_socket_ && client_socket_->impl()->initialized();
}

bool PLCCommunicator::sendAndReceive(const std::array<uint8_t, IO_DATA_SIZE>& send_data, std::array<uint8_t, IO_DATA_SIZE>& recv_data)
{
    if (!isClientConnected()) {
        setLastError("No PLC client connected");
        return false;
    }

    auto start_time = std::chrono::steady_clock::now();

    try {


        // 64바이트 수신 대기 (블로킹)
       // LOG_DEBUG("Waiting for 64 bytes response from PLC client...");
       if (!receiveDataBlocking(recv_data)) {
            failed_transactions_++;
            handleClientDisconnection();
            return false;
        }

        // === 수신 로그 부분 ===
        std::string recvdump;
        Poco::Logger::formatDump(recvdump, recv_data.data(), recv_data.size());

        auto now_recv = std::chrono::steady_clock::now();
        double elapsed_recv_ms = std::chrono::duration<double, std::milli>(now_recv - last_recv_log_time_).count();

        if (elapsed_recv_ms >= 100.0) // 0.1초 주기
        {
            // NLOG(info) << "[RECV] " << recvdump;
            last_recv_log_time_ = now_recv;
        }

        // 64바이트 송신 (블로킹)
       // LOG_DEBUG("Sending 64 bytes to PLC client...");
        if (!sendDataBlocking(send_data)) {
            failed_transactions_++;
            handleClientDisconnection();
            return false;
        }

        // === 송신 로그 부분 ===
        std::string dump;
        Poco::Logger::formatDump(dump, send_data.data(), send_data.size());

        auto now_send = std::chrono::steady_clock::now();
        double elapsed_send_ms = std::chrono::duration<double, std::milli>(now_send - last_send_log_time_).count();

        if (elapsed_send_ms >= 100.0) // 0.1초 주기
        {
            // NLOG(info) << "[SEND] " << dump;
            last_send_log_time_ = now_send;
        }

        


        // 마지막 입력 데이터 저장
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            last_input_ = recv_data;
        }

        auto end_time = std::chrono::steady_clock::now();
        double transaction_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        updateStats(transaction_time);
        successful_transactions_++;

        // LOG_DEBUG("PLC transaction completed in %.2f ms", transaction_time);
        return true;
    }
    catch (const Poco::TimeoutException& ex) {
        setLastError("Communication timeout: " + ex.displayText());
        failed_transactions_++;
        handleClientDisconnection();
        return false;
    }
    catch (const Poco::Net::ConnectionResetException& ex) {
        setLastError("Connection reset by PLC client: " + ex.displayText());
        failed_transactions_++;
        handleClientDisconnection();
        return false;
    }
    catch (const Poco::Exception& ex) {
        setLastError("Communication error: " + ex.displayText());
        failed_transactions_++;
        handleClientDisconnection();
        return false;
    }
}

void PLCCommunicator::setOutputData(const std::array<uint8_t, IO_DATA_SIZE>& output_data)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_output_ = output_data;
}

std::array<uint8_t, PLCCommunicator::IO_DATA_SIZE> PLCCommunicator::getOutputData() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_output_;
}

std::array<uint8_t, PLCCommunicator::IO_DATA_SIZE> PLCCommunicator::getLastInputData() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_input_;
}

bool PLCCommunicator::createServerSocket()
{
    try {
        Poco::Net::SocketAddress addr(config_.bind_ip, config_.port);
        server_socket_ = std::make_unique<Poco::Net::ServerSocket>(addr);
        configureServerSocket();
        return true;
    }
    catch (const Poco::Exception& ex) {
        setLastError("Failed to create server socket: " + ex.displayText());
        return false;
    }
}

void PLCCommunicator::configureServerSocket()
{
    if (!server_socket_)
        return;

    try {
        // SO_REUSEADDR 설정
        if (config_.reuse_address) {
            server_socket_->setReuseAddress(true);
        }

        // 백로그 설정 (대기 큐 크기)
        server_socket_->listen(config_.max_connections);

        LOG_DEBUG(
            "Server socket configured: reuse_addr=%s, max_connections=%d", config_.reuse_address ? "true" : "false",
            config_.max_connections);
    }
    catch (const Poco::Exception& ex) {
        LOG_WARNING("Failed to configure server socket options: %s", ex.displayText().c_str());
    }
}

void PLCCommunicator::acceptLoop()
{
    LOG_INFO("Accept loop started, waiting for PLC connections...");

    while (!should_stop_ && server_state_ != ServerState::STOPPED) {
        try {
            if (acceptClient()) {
                setState(ServerState::CONNECTED);
                LOG_INFO("PLC client connected: %s", connected_client_ip_.c_str());

                // 클라이언트가 연결되어 있는 동안 대기
                while (!should_stop_ && isClientConnected()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                if (!should_stop_) {
                    LOG_WARNING("PLC client disconnected: %s", connected_client_ip_.c_str());
                    setState(ServerState::LISTENING);
                }
            }
        }
        catch (const Poco::Exception& ex) {
            if (!should_stop_) {
                LOG_ERROR("Error in accept loop: %s", ex.displayText().c_str());
                setState(ServerState::ERROR);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }

    LOG_INFO("Accept loop finished");
}

bool PLCCommunicator::acceptClient()
{
    if (!server_socket_) {
        return false;
    }

    try {
        // 논블로킹 accept를 위한 타임아웃 설정
        Poco::Timespan timeout(config_.accept_timeout_ms * 1000);

        if (server_socket_->poll(timeout, Poco::Net::Socket::SELECT_READ)) {
            auto new_client = std::make_unique<Poco::Net::StreamSocket>();
            *new_client = server_socket_->acceptConnection();

            // 클라이언트 정보 저장
            connected_client_ip_ = new_client->peerAddress().toString();

            // 클라이언트 소켓 설정
            configureClientSocket(*new_client);

            // 기존 클라이언트가 있으면 해제
            {
                std::lock_guard<std::mutex> lock(client_mutex_);
                if (client_socket_) {
                    try {
                        client_socket_->close();
                    }
                    catch (...) {
                    }
                }
                client_socket_ = std::move(new_client);
            }

            return true;
        }

        return false;  // 타임아웃
    }
    catch (const Poco::Exception& ex) {
        setLastError("Failed to accept client: " + ex.displayText());
        return false;
    }
}

void PLCCommunicator::handleClientDisconnection()
{
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (client_socket_) {
        try {
            client_socket_->close();
        }
        catch (...) {
        }
        client_socket_.reset();
    }

    if (server_state_ == ServerState::CONNECTED) {
        setState(ServerState::LISTENING);
        LOG_WARNING("Client disconnected, waiting for reconnection...");
    }
}

void PLCCommunicator::configureClientSocket(Poco::Net::StreamSocket& socket)
{
    try {
        // 블로킹 모드 설정
        socket.setBlocking(true);

        // TCP_NODELAY (낮은 지연시간)
        if (config_.no_delay) {
            socket.setNoDelay(true);
        }

        // Keep-Alive
        if (config_.keep_alive) {
            socket.setKeepAlive(true);
        }

        // 타임아웃 설정
        Poco::Timespan send_timeout(config_.send_timeout_ms * 1000);
        Poco::Timespan recv_timeout(config_.receive_timeout_ms * 1000);
        socket.setSendTimeout(send_timeout);
        socket.setReceiveTimeout(recv_timeout);

        // 버퍼 크기 설정
        socket.setSendBufferSize(config_.socket_buffer_size);
        socket.setReceiveBufferSize(config_.socket_buffer_size);

        // Linger 설정
        socket.setLinger(true, 2);

        LOG_DEBUG(
            "Client socket configured: blocking=true, nodelay=%s, keepalive=%s", config_.no_delay ? "true" : "false",
            config_.keep_alive ? "true" : "false");
    }
    catch (const Poco::Exception& ex) {
        LOG_WARNING("Failed to configure client socket options: %s", ex.displayText().c_str());
    }
}

bool PLCCommunicator::sendDataBlocking(const std::array<uint8_t, IO_DATA_SIZE>& data)
{
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_socket_) {
        return false;
    }

    try {
        int total_sent = 0;

        while (total_sent < IO_DATA_SIZE) {
            int bytes_sent = client_socket_->sendBytes(data.data() + total_sent, IO_DATA_SIZE - total_sent);

            if (bytes_sent <= 0) {
                setLastError("Send failed: socket closed or error");
                return false;
            }

            total_sent += bytes_sent;
        }

        return true;
    }
    catch (const Poco::TimeoutException& ex) {
        setLastError("Send timeout: " + ex.displayText());
        return false;
    }
    catch (const Poco::Exception& ex) {
        setLastError("Send failed: " + ex.displayText());
        return false;
    }
}

bool PLCCommunicator::receiveDataBlocking(std::array<uint8_t, IO_DATA_SIZE>& data)
{
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_socket_) {
        return false;
    }

    try {
        int total_received = 0;
        data.fill(0);
        
        while (total_received < IO_DATA_SIZE) {
            // LOG_INFO("CLIENT RECEIVE: %d/%d", total_received, IO_DATA_SIZE);
            int bytes_received = client_socket_->receiveBytes(data.data() + total_received, IO_DATA_SIZE - total_received);
            // LOG_INFO("CLIENT RECEIVE: %d/%d/%d", total_received, IO_DATA_SIZE, bytes_received);
        

            if (bytes_received <= 0) {
                setLastError("Receive failed: socket closed or error");
                return false;
            }

            total_received += bytes_received;
        }

        return true;
    }
    catch (const Poco::TimeoutException& ex) {
        setLastError("Receive timeout: " + ex.displayText());
        return false;
    }
    catch (const Poco::Exception& ex) {
        setLastError("Receive failed: " + ex.displayText());
        return false;
    }
}

PLCCommunicator::ServerState PLCCommunicator::getServerState() const
{
    return server_state_;
}

std::string PLCCommunicator::getServerStateString() const
{
    switch (server_state_) {
        case ServerState::STOPPED:
            return "STOPPED";
        case ServerState::LISTENING:
            return "LISTENING";
        case ServerState::CONNECTED:
            return "CONNECTED";
        case ServerState::ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

std::string PLCCommunicator::getConnectedClientIP() const
{
    return connected_client_ip_;
}

void PLCCommunicator::updateStats(double transaction_time)
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    transaction_times_.push_back(transaction_time);

    // 최근 100개 트랜잭션만 유지
    if (transaction_times_.size() > 100) {
        transaction_times_.erase(transaction_times_.begin());
    }
}

void PLCCommunicator::setState(ServerState state)
{
    server_state_ = state;
    LOG_DEBUG("Server state changed to: %s", getServerStateString().c_str());
}

std::string PLCCommunicator::getLastError() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return last_error_;
}

PLCCommunicator::ServerConfig PLCCommunicator::getConfig() const
{
    return config_;
}

size_t PLCCommunicator::getSuccessfulTransactions() const
{
    return successful_transactions_;
}

size_t PLCCommunicator::getFailedTransactions() const
{
    return failed_transactions_;
}

double PLCCommunicator::getAverageTransactionTime() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (transaction_times_.empty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (double time : transaction_times_) {
        sum += time;
    }
    return sum / transaction_times_.size();
}

void PLCCommunicator::setLastError(const std::string& error)
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    last_error_ = error;
    LOG_ERROR("PLC Server Error: %s", error.c_str());
}

}  // namespace NaviFra