#include "interface/can/can_driver_base.hpp"

#include "core/util/logger.hpp"

using namespace NaviFra;

void Can_driver::InterfaceOpen()
{
    mlockall(MCL_CURRENT | MCL_FUTURE);

    Init();

    if (false == th_readLoop_.joinable()) {
        th_readLoop_ = std::thread(&Can_driver::Read_Loop, this);
    }
}

void Can_driver::Init()
{
    if ((n_can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
    }
    const char* can_dev = "can1";

    strcpy(can_ifr.ifr_name, can_dev);
    ioctl(n_can_fd_, SIOCGIFINDEX, &can_ifr);
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = can_ifr.ifr_ifindex;
    fcntl(n_can_fd_, F_SETFL, O_NONBLOCK);

    if (bind(n_can_fd_, reinterpret_cast<struct sockaddr*>(&can_addr), sizeof(can_addr)) < 0) {
        LOG_ERROR("CAN Binding ERROR");
        exit(1);
    }
}

void Can_driver::uninit()
{
    close(n_can_fd_);
}

ssize_t Can_driver::safe_CAN_Write(int n_can_fd, can_frame* msg)
{
    std::lock_guard<std::mutex> lock(rw_mtx);  // 유일 잠금 (쓰기)
    return write(n_can_fd, msg, sizeof(*msg));  // 메시지 쓰기
    // 잠금 해제는 lock이 소멸될 때 자동으로 이루어짐
}

ssize_t Can_driver::safe_CAN_Read(int n_can_fd, can_frame* Message)
{
    std::unique_lock<std::mutex> lock(rw_mtx, std::try_to_lock);
    if (lock.owns_lock()) {
        return read(n_can_fd, Message, sizeof(*Message));  // Only read if lock was successful
    }
    else {
        // Handle the case where the mutex is locked, e.g., try again later
        // This prevents blocking and potential read failures due to timing
        return 0;
    }
}

void Can_driver::Write(int n_can_id, uint8_t* un8_send_data, uint8_t un8_data_length)
{
    un8_data_length = 8 < un8_data_length ? 8 : un8_data_length;
    can_frame msg;
    msg.can_id = n_can_id;
    msg.can_dlc = un8_data_length;
    memcpy(msg.data, un8_send_data, un8_data_length);
    for (int i = un8_data_length; i < 8; i++) {
        msg.data[i] = 0;
    }
    Write(msg);
}

void Can_driver::Write(
    int n_can_id, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7, uint8_t byte8)
{
    uint8_t un8_data[8] = {byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8};
    Write(n_can_id, un8_data, 8);
}

void Can_driver::Write(can_frame msg)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    // n_bytes_write_ = write(n_can_fd_, &msg, sizeof(msg));
    n_bytes_write_ = safe_CAN_Write(n_can_fd_, &msg);
    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start_time;
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    if (n_bytes_write_ < 1 || sec.count() * 1000 > 3) {
        perror("can_write()");
        LOG_ERROR("CANWRITE: %d byte, %f ms", n_bytes_write_, sec.count() * 1000);
    }
    else {
        Notify("can_callback", msg);
    }
}

void reset()
{
}

void Can_driver::Read_Loop()
{
    unsigned int n_pre_id;
    while (1) {
        can_frame Message;
        can_err_mask_t err_mask;
        err_mask = CAN_ERR_FLAG;

        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        // n_nbytes_read_ = read(n_can_fd_, &Message, sizeof(Message));
        n_nbytes_read_ = safe_CAN_Read(n_can_fd_, &Message);
        std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start_time;

        if (n_nbytes_read_ < 1) {
            // can 데이터가 없을 때 슬립 걸어주기
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
        else {
            Notify("CanCallback", Message);
            Notify("can_callback", Message);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        if (sec.count() * 1000 > 3) {
            LOG_ERROR("CANREAD: %f ms, now id %x / pre id %x", sec.count() * 1000, Message.can_id, n_pre_id);
        }
        n_pre_id = Message.can_id;

        if (Message.can_id & CAN_ERR_FLAG) {
            string s_can_error_text = Can_error_check(Message.can_id & CAN_ERR_MASK);
            LOG_ERROR("CAN ERROR : %s", s_can_error_text.c_str());
        }
    }
}

bool Can_driver::Notify(const std::string& str_cbf_name, const boost::any& any_type_var)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end()) {
        return false;
    }
    // std::async(std::launch::async, [this, str_cbf_name, any_type_var]() {
    //     std::string name_copy = str_cbf_name;
    //     boost::any any_copy = any_type_var;
    //     map_callback_pt_[name_copy](any_copy);
    // });
    map_callback_pt_[str_cbf_name](any_type_var);
    return true;
}

void Can_driver::WriteCan(const int& id, const vector<int>& data)
{
    Write(id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

string Can_driver::Can_error_check(unsigned int error_code)
{
    string s_can_error_text = "";
    if (error_code & CAN_ERR_TX_TIMEOUT)
        s_can_error_text = "TX Timeout";
    if (error_code & CAN_ERR_LOSTARB)
        s_can_error_text = "Lost Arbitration";
    if (error_code & CAN_ERR_CRTL)
        s_can_error_text = "Controller Problems";
    if (error_code & CAN_ERR_PROT)
        s_can_error_text = "Protocol Violations";
    if (error_code & CAN_ERR_TRX)
        s_can_error_text = "Transceiver Status";
    if (error_code & CAN_ERR_ACK)
        s_can_error_text = "No ACK on Transmission";
    if (error_code & CAN_ERR_BUSOFF)
        s_can_error_text = "Bus Off";
    if (error_code & CAN_ERR_BUSERROR)
        s_can_error_text = "Bus Error";
    if (error_code & CAN_ERR_RESTARTED)
        s_can_error_text = "Controller Restarted";
    return s_can_error_text;
}

bool Can_driver::RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end()) {
        map_callback_pt_[str_cbf_name] = pt_func;
        return true;
    }
    else {
        return false;
    }
}
MotorState Can_driver::handleMotorControl(int motorStatus)
{
    MotorState state;
    switch (motorStatus & 0x4F) {
        case STATUS_NOT_READY:
            state = STATE_SWITCH_ON_DISABLE;
            break;

        case STATUS_SWITCH_ON_DISABLED:
            state = STATE_SWITCH_ON_DISABLE;
            break;

        case STATUS_READY_SWITCH_ON:
            state = STATE_READY_TO_SWITCH_ON;
            break;

        case STATUS_SWITCHED_ON:
            state = STATE_SWITCH_ON;
            break;

        case STATUS_OP_ENABLED:
            state = STATE_OPERATION_ENABLE;
            break;

        case STATUS_FAULT:
            // STATUS_FAULT 상태일 때 CONTROL_WORD_FAULT_RESET 명령 전송
            state = STATE_RESET_MOTOR;
            break;

        default:
            // 알 수 없는 상태일 경우 처리
            // state = STATE_RESET_MOTOR;
            LOG_ERROR("Unknown motor status: %d", motorStatus);
            break;
    }
    LOG_INFO("###Motor state: %d", state);
    return state;
}