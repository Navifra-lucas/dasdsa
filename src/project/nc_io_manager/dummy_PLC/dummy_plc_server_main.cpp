#include <iostream>
#include <thread>
#include <atomic>
#include <array>
#include <vector>
#include <chrono>
#include <memory>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <signal.h>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static constexpr size_t IO_DATA_SIZE = 64;
static std::atomic<bool> g_running{true};

// 시그널 핸들러
void signalHandler(int signal) {
    std::cout << "\n[MAIN] Received signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
}

// PLC 상태 시뮬레이션 클래스
class PLCSimulator {
public:
    PLCSimulator() {
        // 입력 데이터 초기화
        input_data_.fill(0);
        output_data_.fill(0);
        
        // 초기 센서 상태 설정
        initializeDefaultStates();
        
        // 시뮬레이션 스레드 시작
        simulation_thread_ = std::thread(&PLCSimulator::simulationLoop, this);
        
        std::cout << "[PLC_SIM] PLC Simulator initialized" << std::endl;
    }
    
    ~PLCSimulator() {
        running_ = false;
        if (simulation_thread_.joinable()) {
            simulation_thread_.join();
        }
        std::cout << "[PLC_SIM] PLC Simulator destroyed" << std::endl;
    }
    
    // 클라이언트로부터 데이터 수신 및 응답
    std::array<uint8_t, IO_DATA_SIZE> processCommand(const std::array<uint8_t, IO_DATA_SIZE>& received_data) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 수신된 출력 데이터 저장
        output_data_ = received_data;
        
        // 출력에 따른 입력 상태 업데이트
        updateInputsBasedOnOutputs();
        
        // 현재 입력 데이터 반환
        return input_data_;
    }
    
    void printStatus() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        std::cout << "\n=== PLC Simulator Status ===" << std::endl;
        
        // 주요 출력 상태
        std::cout << "Outputs:" << std::endl;
        std::cout << "  LIFT: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(output_data_[20]) << std::dec << std::endl;
        std::cout << "  LIFT_TURN: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(output_data_[21]) << std::dec << std::endl;
        std::cout << "  CONVEYOR: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(output_data_[22]) << std::dec << std::endl;
        
        // 주요 입력 상태
        std::cout << "Inputs:" << std::endl;
        std::cout << "  SENSOR1: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(input_data_[16]) << std::dec << std::endl;
        std::cout << "  SENSOR2: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(input_data_[17]) << std::dec << std::endl;
        std::cout << "  LIFT: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(input_data_[20]) << std::dec << std::endl;
        std::cout << "  LIFT_TURN: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(input_data_[22]) << std::dec << std::endl;
        
        std::cout << "===========================\n" << std::endl;
    }

private:
    std::array<uint8_t, IO_DATA_SIZE> input_data_;
    std::array<uint8_t, IO_DATA_SIZE> output_data_;
    mutable std::mutex data_mutex_;
    
    // 시뮬레이션 상태
    std::atomic<bool> running_{true};
    std::thread simulation_thread_;
    
    // 리프트 시뮬레이션 상태
    enum class LiftState { DOWN, MOVING_UP, UP, MOVING_DOWN } lift_state_ = LiftState::DOWN;
    std::chrono::steady_clock::time_point lift_move_start_;
    static constexpr int LIFT_MOVE_TIME_MS = 8000; // 3초
    
    void initializeDefaultStates() {
        // 기본 안전 상태 설정
        input_data_[8] = 0x00;  // SAFETY - 모든 안전 신호 OK
        input_data_[10] = 0x00; // BUTTON - 버튼 상태
        input_data_[13] = 0x3C; // STATE_INFO - 기본 상태
        
        // 센서 초기 상태
        input_data_[16] = 0x03; // SENSOR1 - FRONT_DOCK, REAR_DOCK ON
        input_data_[17] = 0x00; // SENSOR2 - 자석 센서들 OFF
        
        // 리프트 센서 - 초기에는 DOWN 상태
        input_data_[20] = 0x03; // LIFT - DOWN_1, DOWN_2 ON
        input_data_[22] = 0x02; // LIFT_TURN - DOWN_FEEDBACK ON
        
        // 컨베이어 상태
        input_data_[25] = 0x00; // CONVEYOR1
        input_data_[26] = 0x00; // CONVEYOR2
        input_data_[27] = 0x00; // CONVEYOR3
        
        std::cout << "[PLC_SIM] Default states initialized" << std::endl;
    }
    
    void updateInputsBasedOnOutputs() {
        // 리프트 제어 시뮬레이션
        uint8_t lift_cmd = output_data_[20]; // LIFT output
        
        if (lift_cmd == 0x01) { // 리프트 업 명령
            if (lift_state_ == LiftState::DOWN) {
                lift_state_ = LiftState::MOVING_UP;
                lift_move_start_ = std::chrono::steady_clock::now();
                
                // 리프트 센서 상태 변경 - 이동 중에는 모든 센서 OFF
                input_data_[20] = 0x00; // LIFT sensors OFF
                input_data_[22] &= ~0x02; // DOWN_FEEDBACK OFF
                
                // 첫 번째 스텝을 위해 FRONT_MAGNET_SENSOR ON

                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                input_data_[17] |= 0x01; // FRONT_MAGNET_SENSOR ON
                
                //std::cout << "[PLC_SIM] Lift moving UP started" << std::endl;
            }
        } else if (lift_cmd == 0x02) { // 리프트 정지/다운 명령
            if (lift_state_ == LiftState::UP) {
                // 세 번째 스텝: UP_FEEDBACK 신호 제공
                input_data_[22] |= 0x01; // UP_FEEDBACK ON
                //std::cout << "[PLC_SIM] Lift UP_FEEDBACK activated" << std::endl;
            } else if (lift_state_ == LiftState::MOVING_DOWN) {
                // 다운 동작 처리
                lift_state_ = LiftState::MOVING_DOWN;
                lift_move_start_ = std::chrono::steady_clock::now();
                
                // 리프트 센서 상태 변경
                input_data_[20] = 0x00; // LIFT sensors OFF
                input_data_[22] &= ~0x01; // UP_FEEDBACK OFF
                
                // std::cout << "[PLC_SIM] Lift moving DOWN started" << std::endl;
            }
        }
    }
    
    void simulationLoop() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            std::lock_guard<std::mutex> lock(data_mutex_);
            updateLiftSimulation();
            updateSensorSimulation();
        }
    }
    
    void updateLiftSimulation() {
        auto now = std::chrono::steady_clock::now();
        
        switch (lift_state_) {
            case LiftState::MOVING_UP: {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - lift_move_start_).count();
                
                if (elapsed >= LIFT_MOVE_TIME_MS) {
                    // 리프트 업 완료
                    lift_state_ = LiftState::UP;
                    input_data_[20] = 0x0C; // UP_1, UP_2 ON (bit 2,3)
                    input_data_[22] |= 0x01; // UP_FEEDBACK ON
                    std::cout << "[PLC_SIM] Lift UP completed" << std::endl;
                }
                break;
            }
            
            case LiftState::MOVING_DOWN: {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - lift_move_start_).count();
                
                if (elapsed >= LIFT_MOVE_TIME_MS) {
                    // 리프트 다운 완료
                    lift_state_ = LiftState::DOWN;
                    input_data_[20] = 0x03; // DOWN_1, DOWN_2 ON (bit 0,1)
                    input_data_[22] |= 0x02; // DOWN_FEEDBACK ON
                    std::cout << "[PLC_SIM] Lift DOWN completed" << std::endl;
                }
                break;
            }
            
            default:
                break;
        }
    }
    
    void updateSensorSimulation() {
        // 주기적으로 센서 상태 업데이트 (시뮬레이션)
        static int counter = 0;
        counter++;
        
        // 리프트 시퀀스 호환성을 위한 센서 시뮬레이션
        // FRONT_MAGNET_SENSOR는 SENSOR2(input_[17])의 비트 0
        if (counter % 30 == 0) { // 100ms * 30 = 3초마다
            input_data_[17] |= 0x01; // FRONT_MAGNET_SENSOR ON (비트 0)
            // std::cout << "[PLC_SIM] Front magnet sensor ON" << std::endl;
        }
        
        // 리프트 동작 중에는 자석 센서를 켜서 첫 번째 스텝 통과하도록 함
        if (lift_state_ == LiftState::MOVING_UP || lift_state_ == LiftState::UP) {
            input_data_[17] |= 0x01; // FRONT_MAGNET_SENSOR 유지
        }
    }
};

// 클라이언트 연결 처리 함수
void handleClient(int client_socket, std::shared_ptr<PLCSimulator> simulator, int client_id) {
    std::cout << "[CLIENT_" << client_id << "] Connected" << std::endl;
    
    int transaction_count = 0;
    
    while (g_running) {
        // 64바이트 수신
        std::array<uint8_t, IO_DATA_SIZE> received_data;
        ssize_t bytes_received = recv(client_socket, received_data.data(), IO_DATA_SIZE, 0);
        
        if (bytes_received <= 0) {
            std::cout << "[CLIENT_" << client_id << "] Connection closed or error" << std::endl;
            break;
        }
        
        if (bytes_received != IO_DATA_SIZE) {
            std::cout << "[CLIENT_" << client_id << "] Incomplete data received: " 
                      << bytes_received << " bytes" << std::endl;
            continue;
        }
        
        // PLC 시뮬레이터에서 처리
        auto response = simulator->processCommand(received_data);
        
        // 64바이트 송신
        ssize_t bytes_sent = send(client_socket, response.data(), IO_DATA_SIZE, 0);
        if (bytes_sent != IO_DATA_SIZE) {
            std::cout << "[CLIENT_" << client_id << "] Send failed: " 
                      << bytes_sent << " bytes" << std::endl;
            break;
        }
        
        // 주기적으로 상태 출력 (100회마다)
        if (++transaction_count % 100 == 0) {
            std::cout << "[CLIENT_" << client_id << "] Transactions: " 
                      << transaction_count << std::endl;
        }
    }
    
    close(client_socket);
    std::cout << "[CLIENT_" << client_id << "] Disconnected" << std::endl;
}

// 메인 서버 클래스
class SimpleServer {
public:
    SimpleServer(int port) : port_(port), server_socket_(-1) {
        simulator_ = std::make_shared<PLCSimulator>();
    }
    
    ~SimpleServer() {
        stop();
    }
    
    bool start() {
        // 소켓 생성
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ < 0) {
            std::cerr << "[SERVER] Failed to create socket" << std::endl;
            return false;
        }
        
        // 소켓 옵션 설정
        int opt = 1;
        if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "[SERVER] Failed to set socket options" << std::endl;
            return false;
        }
        
        // 주소 설정
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port_);
        
        // 바인드
        if (bind(server_socket_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cerr << "[SERVER] Failed to bind to port " << port_ << std::endl;
            return false;
        }
        
        // 리슨
        if (listen(server_socket_, 5) < 0) {
            std::cerr << "[SERVER] Failed to listen" << std::endl;
            return false;
        }
        
        std::cout << "[SERVER] Simple PLC Server started on port " << port_ << std::endl;
        std::cout << "[SERVER] Waiting for connections..." << std::endl;
        
        // 상태 출력 스레드 시작
        status_thread_ = std::thread(&SimpleServer::statusLoop, this);
        
        // 클라이언트 연결 처리
        int client_id = 0;
        while (g_running) {
            try {
                // select()를 사용해서 1초 타임아웃으로 연결 요청 확인
                fd_set readfds;
                FD_ZERO(&readfds);
                FD_SET(server_socket_, &readfds);
                
                struct timeval timeout;
                timeout.tv_sec = 1;   // 1초 타임아웃
                timeout.tv_usec = 0;
                
                int result = select(server_socket_ + 1, &readfds, NULL, NULL, &timeout);
                
                if (result == -1) {
                    if (g_running) {
                        std::cerr << "[SERVER] Select error" << std::endl;
                    }
                    break;
                } else if (result == 0) {
                    // 타임아웃 - g_running 체크하고 계속
                    continue;
                } else {
                    // 연결 요청이 있음
                    struct sockaddr_in client_addr;
                    socklen_t client_len = sizeof(client_addr);
                    
                    int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
                    if (client_socket < 0) {
                        if (g_running) {
                            std::cerr << "[SERVER] Accept failed" << std::endl;
                        }
                        continue;
                    }
                    
                    if (!g_running) {
                        close(client_socket);
                        break;
                    }
                    
                    // 클라이언트 처리
                    ++client_id;
                    client_threads_.emplace_back(handleClient, client_socket, simulator_, client_id);
                    cleanupFinishedThreads();
                }
            }
            catch (const std::exception& ex) {
                if (g_running) {
                    std::cout << "[SERVER] Exception: " << ex.what() << std::endl;
                }
                break;
            }
        }
        
        return true;
    }
    
    void stop() {
        g_running = false;
        
        if (server_socket_ >= 0) {
            close(server_socket_);
            server_socket_ = -1;
        }
        
        // 모든 클라이언트 스레드 종료 대기
        for (auto& thread : client_threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        client_threads_.clear();
        
        if (status_thread_.joinable()) {
            status_thread_.join();
        }
        
        std::cout << "[SERVER] Server stopped" << std::endl;
    }

private:
    int port_;
    int server_socket_;
    std::shared_ptr<PLCSimulator> simulator_;
    std::vector<std::thread> client_threads_;
    std::thread status_thread_;
    
    void statusLoop() {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::seconds(10));
            if (g_running) {
                simulator_->printStatus();
            }
        }
    }
    
    void cleanupFinishedThreads() {
        auto it = client_threads_.begin();
        while (it != client_threads_.end()) {
            if (it->joinable()) {
                ++it;
            } else {
                it = client_threads_.erase(it);
            }
        }
    }
};

int main(int argc, char** argv) {
    // 시그널 핸들러 등록
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    int port = 5000; // 기본 포트
    
    if (argc > 1) {
        port = std::atoi(argv[1]);
        if (port <= 0 || port > 65535) {
            std::cout << "Invalid port number. Using default port 5000" << std::endl;
            port = 5000;
        }
    }
    
    std::cout << "=== Simple Dummy PLC Server ===" << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "Use Ctrl+C to stop" << std::endl;
    std::cout << "===============================" << std::endl;
    
    try {
        SimpleServer server(port);
        if (!server.start()) {
            return -1;
        }
    }
    catch (const std::exception& ex) {
        std::cout << "[MAIN] Exception: " << ex.what() << std::endl;
        return -1;
    }
    
    std::cout << "[MAIN] Server shutdown completed" << std::endl;
    return 0;
}