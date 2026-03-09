#include "NaviCAT/NaviCATCore.h"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

using namespace NaviFra::NaviCAT;

bool NaviCATCore::initialize()
{
    return true;
}

bool NaviCATCore::initialize(const std::string& interface, const std::string& yaml_file)
{
    // YAML에서 PDO 매핑 정보 로드
    pdo_mapper_ = std::make_unique<PDOMapper>();
    if (!pdo_mapper_->loadFromYAML(yaml_file)) {
        std::cerr << "YAML 파일 로드 실패: " << yaml_file << std::endl;
        return false;
    }

    pdo_mapper_->printMapping();

    // PDO 데이터 객체 생성
    outputs_ = std::make_unique<PDOData>(pdo_mapper_->getTxPDOSize(), 
                                        pdo_mapper_->getTxPDOEntries());
    inputs_ = std::make_unique<PDOData>(pdo_mapper_->getRxPDOSize(), 
                                         pdo_mapper_->getRxPDOEntries());

    // 어댑터 목록 조회
    adapter_list_ = ec_find_adapters();
    if (adapter_list_ == nullptr) {
        std::cerr << "네트워크 어댑터를 찾을 수 없습니다!" << std::endl;
        return false;
    }

    // 지정된 인터페이스 찾기
    ec_adaptert* adapter = adapter_list_;
    bool found = false;
    while (adapter != nullptr) {
        if (interface == adapter->name) {
            found = true;
            break;
        }
        adapter = adapter->next;
    }

    if (!found) {
        std::cerr << "지정된 인터페이스를 찾을 수 없음: " << interface << std::endl;
        ec_free_adapters(adapter_list_);
        return false;
    }

    // EtherCAT 마스터 초기화 (SOEM v2.0.0)
    if (ecx_init(&ecx_context_, const_cast<char*>(interface.c_str())) <= 0) {
        std::cerr << "EtherCAT 초기화 실패: " << interface << std::endl;
        ec_free_adapters(adapter_list_);
        return false;
    }

    std::cout << "EtherCAT 초기화 성공: " << interface << std::endl;

    // 슬레이브 검색 및 설정 
    if (ecx_config_init(&ecx_context_) <= 0) {
        std::cerr << "슬레이브를 찾을 수 없습니다!" << std::endl;
        return false;
    }

    std::cout << ecx_context_.slavecount << "개의 슬레이브를 발견했습니다." << std::endl;

    // 슬레이브별 설정
    ecx_context_.userdata = this;
    for (int cnt = 1; cnt <= ecx_context_.slavecount; cnt++) {
        // setupSlave(cnt);
    }

    // PDO 매핑 및 상태 전환 (SOEM v2.0.0)
    ecx_config_map_group(&ecx_context_, &IOmap_, 0);
    ecx_configdc(&ecx_context_);
        
    std::cout << "슬레이브 매핑 완료, SAFE_OP 상태로 전환" << std::endl;
    ecx_statecheck(&ecx_context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
        
    std::cout << "모든 슬레이브에 대해 OPERATIONAL 상태 요청" << std::endl;
    ecx_context_.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_send_processdata(&ecx_context_);
    ecx_receive_processdata(&ecx_context_, EC_TIMEOUTRET);
    ecx_writestate(&ecx_context_, 0);
        
    ecx_statecheck(&ecx_context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        
    if (ecx_context_.slavelist[0].state != EC_STATE_OPERATIONAL) {
        std::cerr << "모든 슬레이브가 OPERATIONAL 상태에 도달하지 못했습니다." << std::endl;
        ecx_readstate(&ecx_context_);
        for(int i = 1; i <= ecx_context_.slavecount; i++) {
            if(ecx_context_.slavelist[i].state != EC_STATE_OPERATIONAL) {
                std::printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                           i, ecx_context_.slavelist[i].state, 
                           ecx_context_.slavelist[i].ALstatuscode,
                           ec_ALstatuscode2string(ecx_context_.slavelist[i].ALstatuscode));
            }
        }
        return false;
    }

    std::cout << "모든 슬레이브가 OPERATIONAL 상태에 도달했습니다." << std::endl;
    inOP_ = true;

    // 슬레이브의 PDO 포인터 연결
    ecx_context_.slavelist[SLAVE_NUM].outputs = outputs_->getData();
    ecx_context_.slavelist[SLAVE_NUM].inputs = inputs_->getData();

    return true;
}

bool NaviCATCore::initializeMotor()
{
    if (!inOP_) return false;
    // 초기 모드 설정
    outputs_->setValue<int8_t>("mode_of_operation", OperationMode::PROFILE_VELOCITY);
    std::cout << "모터 초기화 중..." << std::endl;

    // 모터 상태 머신 실행
    for (int i = 0; i < 100; i++) {
        ecx_send_processdata(&ecx_context_);
        wkc_ = ecx_receive_processdata(&ecx_context_, EC_TIMEOUTRET);
        
        // inputs 데이터 업데이트
        std::memcpy(inputs_->getData(), ecx_context_.slavelist[SLAVE_NUM].inputs, inputs_->getSize());
        
        motorStateMachine();
        
        // outputs 데이터 전송
        std::memcpy(ecx_context_.slavelist[SLAVE_NUM].outputs, outputs_->getData(), outputs_->getSize());
        
        if ((inputs_->getValue<uint16_t>("statusword") & 0x006F) == DS402::STATUS_OPERATION_ENABLED) {
            std::cout << "모터가 동작 준비 완료!" << std::endl;
            return true;
        }
        
        std::this_thread::sleep_for(10ms);
    }
    std::cerr << "모터 초기화 실패!" << std::endl;
    return false;
}

void NaviCATCore::runJogControl() {
    if (!inOP_) return;

    int32_t jog_velocity = 1000;  // RPM 단위
    running_ = true;
    int loop_count = 0;

    std::cout << "\n조그 제어 시작 (q: 종료, w: 정방향, s: 역방향, x: 정지)" << std::endl;
    std::cout << "현재 속도: " << jog_velocity << " RPM" << std::endl;
    std::cout << "속도 조절: +/- 키 사용" << std::endl;

    while (running_) {
        ecx_send_processdata(&ecx_context_);
        wkc_ = ecx_receive_processdata(&ecx_context_, EC_TIMEOUTRET);
        
        // inputs 데이터 업데이트
        std::memcpy(inputs_->getData(), ecx_context_.slavelist[SLAVE_NUM].inputs, inputs_->getSize());
        
        // 키보드 입력 확인
        if (kbhit()) {
            char direction = getchar();
            
            switch (direction) {
                case 'w':
                case 'W':
                    outputs_->setValue<int32_t>("target_velocity", jog_velocity);
                    std::cout << "정방향 조그: " << jog_velocity << " RPM" << std::endl;
                    break;
                    
                case 's':
                case 'S':
                    outputs_->setValue<int32_t>("target_velocity", -jog_velocity);
                    std::cout << "역방향 조그: " << jog_velocity << " RPM" << std::endl;
                    break;
                    
                case 'x':
                case 'X':
                    outputs_->setValue<int32_t>("target_velocity", 0);
                    std::cout << "정지" << std::endl;
                    break;
                    
                case '+':
                    jog_velocity += 100;
                    if (jog_velocity > 3000) jog_velocity = 3000;
                    std::cout << "속도 증가: " << jog_velocity << " RPM" << std::endl;
                    break;
                    
                case '-':
                    jog_velocity -= 100;
                    if (jog_velocity < 100) jog_velocity = 100;
                    std::cout << "속도 감소: " << jog_velocity << " RPM" << std::endl;
                    break;
                    
                case 'q':
                case 'Q':
                    running_ = false;
                    break;
            }
        }
        
        // 상태 정보 출력 (1초마다)
        if (loop_count % 100 == 0) {
            std::printf("위치: %d, 속도: %d, 토크: %d, 상태: 0x%04X, WKC: %d\n",
                       inputs_->getValue<int32_t>("position_actual"),
                       inputs_->getValue<int32_t>("velocity_actual"), 
                       inputs_->getValue<int16_t>("torque_actual"),
                       inputs_->getValue<uint16_t>("statusword"),
                       wkc_.load());
        }
        
        // outputs 데이터 전송
        std::memcpy(ecx_context_.slavelist[SLAVE_NUM].outputs, outputs_->getData(), outputs_->getSize());
        
        std::this_thread::sleep_for(10ms);
        loop_count++;
    }
}


void NaviCATCore::cleanup() {
    std::cout << "모터 정지 중..." << std::endl;
        
    if (inOP_ && outputs_) {
        outputs_->setValue<int32_t>("target_velocity", 0);
        outputs_->setValue<uint16_t>("controlword", DS402::CTRL_DISABLE_OPERATION);
        
        for (int i = 0; i < 10; i++) {
            std::memcpy(ecx_context_.slavelist[SLAVE_NUM].outputs, outputs_->getData(), outputs_->getSize());
            ecx_send_processdata(&ecx_context_);
            ecx_receive_processdata(&ecx_context_, EC_TIMEOUTRET);
            std::this_thread::sleep_for(10ms);
        }
    }
    
    if (inOP_) {
        std::cout << "모든 슬레이브를 INIT 상태로 전환" << std::endl;
        ecx_context_.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&ecx_context_, 0);
        
        std::cout << "EtherCAT 소켓 종료" << std::endl;
        ecx_close(&ecx_context_);
        inOP_ = false;
    }

    // 어댑터 리스트 정리
    if (adapter_list_) {
        ec_free_adapters(adapter_list_);
        adapter_list_ = nullptr;
    }
}

bool NaviCATCore::finalize() {
    return true;
}

bool NaviCATCore::kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF) {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

void NaviCATCore::motorStateMachine() {
    uint16_t status = inputs_->getValue<uint16_t>("statusword") & 0x006F;
    
    switch(status) {
        case DS402::STATUS_SWITCH_ON_DISABLED:
            std::cout << "State: Switch On Disabled -> Shutdown" << std::endl;
            outputs_->setValue<uint16_t>("controlword", DS402::CTRL_SHUTDOWN);
            break;
            
        case DS402::STATUS_READY_TO_SWITCH_ON:
            std::cout << "State: Ready to Switch On -> Switch On" << std::endl;
            outputs_->setValue<uint16_t>("controlword", DS402::CTRL_SWITCH_ON);
            break;
            
        case DS402::STATUS_SWITCHED_ON:
            std::cout << "State: Switched On -> Enable Operation" << std::endl;
            outputs_->setValue<uint16_t>("controlword", DS402::CTRL_ENABLE_OPERATION);
            break;
            
        case DS402::STATUS_OPERATION_ENABLED:
            std::cout << "State: Operation Enabled" << std::endl;
            break;
            
        case DS402::STATUS_FAULT:
            std::cout << "State: Fault -> Fault Reset" << std::endl;
            outputs_->setValue<uint16_t>("controlword", DS402::CTRL_FAULT_RESET);
            break;
            
        default:
            std::printf("Unknown state: 0x%04X\n", status);
            outputs_->setValue<uint16_t>("controlword", DS402::CTRL_SHUTDOWN);
            break;
    }
}


void NaviCATCore::printAdapters() {
    if (!adapter_list_) {
        adapter_list_ = ec_find_adapters();
    }
    
    if (adapter_list_) {
        std::cout << "사용 가능한 네트워크 어댑터:" << std::endl;
        ec_adaptert* adapter = adapter_list_;
        while (adapter != nullptr) {
            std::printf("- %s (%s)\n", adapter->name, adapter->desc);
            adapter = adapter->next;
        }
    } else {
        std::cout << "사용 가능한 네트워크 어댑터가 없습니다." << std::endl;
    }
}