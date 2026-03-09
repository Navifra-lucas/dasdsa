#include "interface/pcan/pcan_driver_base.hpp"

using namespace NaviFra;

#ifdef NAVIFRA_PEAK_CAN_USB_FOUND
void Pcan_driver::InterfaceOpen()
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    // TPCANBitrateFD str_br =
    // "f_clock=80000000,nom_brp=10,nom_tseg1=5,nom_tseg2=2,nom_sjw=1,data_brp=4,data_tseg1=7,data_tseg2=2,data_sjw=1";
    // st_interface_param_.st_pcan_param.n_pcan_baudrate = PCAN_BAUDRATE::NC_PCAN_BAUD_250K;
    status_ = CAN_Initialize(st_interface_param_.st_pcan_param.n_pcan_bus, st_interface_param_.st_pcan_param.n_pcan_baudrate, 0, 0, 0);
    // status_ = CAN_InitializeFD(st_interface_param_.st_pcan_param.n_pcan_bus, str_br);

    LOG_INFO(
        "CAN_Initialize: bus %x, baud %d, status 0x%x ", st_interface_param_.st_pcan_param.n_pcan_bus,
        st_interface_param_.st_pcan_param.n_pcan_baudrate, (int)status_);

    if (status_)
        LOG_ERROR("PCAN Initialize Fail");

    if (false == th_readLoop_.joinable()) {
        th_readLoop_ = std::thread(&Pcan_driver::Read_Loop, this);
    }
}

void Pcan_driver::uninit()
{
    status_ = CAN_Reset(st_interface_param_.st_pcan_param.n_pcan_bus);
    status_ = CAN_Uninitialize(st_interface_param_.st_pcan_param.n_pcan_bus);
    status_ = CAN_Initialize(st_interface_param_.st_pcan_param.n_pcan_bus, st_interface_param_.st_pcan_param.n_pcan_baudrate, 0, 0, 0);

    LOG_INFO(
        "CAN_Initialize: bus %x, baud %d, status 0x%x ", st_interface_param_.st_pcan_param.n_pcan_bus,
        st_interface_param_.st_pcan_param.n_pcan_baudrate, (int)status_);

    if (status_)
        LOG_ERROR("PCAN Initialize Fail");
}

void Pcan_driver::reset()
{
    status_ = CAN_Reset(st_interface_param_.st_pcan_param.n_pcan_bus);
}

TPCANStatus Pcan_driver::safe_CAN_Write(TPCANHandle Channel, TPCANMsg* MessageBuffer)
{
    std::lock_guard<std::mutex> lock(rw_mtx);  // 유일 잠금 (쓰기)
    return CAN_Write(Channel, MessageBuffer);  // 메시지 쓰기
    // 잠금 해제는 lock이 소멸될 때 자동으로 이루어짐
}

TPCANStatus Pcan_driver::safe_CAN_Read(TPCANHandle Channel, TPCANMsg* MessageBuffer, TPCANTimestamp* TimestampBuffer)
{
    std::unique_lock<std::mutex> lock(rw_mtx, std::try_to_lock);
    if (lock.owns_lock()) {
        return CAN_Read(Channel, MessageBuffer, TimestampBuffer);  // Only read if lock was successful
    }
    else {
        // Handle the case where the mutex is locked, e.g., try again later
        // This prevents blocking and potential read failures due to timing
        return -1;
    }
}

void Pcan_driver::Write(int id, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7, uint8_t b8)
{
    uint8_t data[8] = {b1, b2, b3, b4, b5, b6, b7, b8};
    Write(id, data, 8);
}

void Pcan_driver::Write(int n_can_id, uint8_t* pun8_msg_data, uint8_t un8_data_len)
{
    uint8_t un8_length = std::min(un8_data_len, uint8_t(8));
    TPCANMsg o_msg;
    o_msg.LEN = un8_length;
    memcpy(o_msg.DATA, pun8_msg_data, un8_length);
    o_msg.ID = n_can_id;
    o_msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    for (int i = un8_length; i < 8; i++)
        o_msg.DATA[i] = 0;
    Write(o_msg);
}

void Pcan_driver::Write(TPCANMsg msg)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    static std::chrono::steady_clock::time_point checktime_recv_ = std::chrono::steady_clock::now();
    TPCANStatus status = CAN_GetStatus(st_interface_param_.st_pcan_param.n_pcan_bus);
    if (status != PCAN_ERROR_OK) {
        LOG_ERROR("Pcan Status Error: %x", status);
        // std::this_thread::sleep_for(std::chrono::microseconds(10));
        // return;
    }
    else if (status == PCAN_ERROR_OK) {
        status = safe_CAN_Write(st_interface_param_.st_pcan_param.n_pcan_bus, &msg);
        if (status != PCAN_ERROR_OK) {
            LOG_ERROR("Tx Error: %x", status);
        }
        else if (status == PCAN_ERROR_OK) {
            checktime_recv_ = std::chrono::steady_clock::now();
            Notify("pcan_callback", msg);
        }
    }
    
    std::chrono::duration<double> sec_check = std::chrono::steady_clock::now() - checktime_recv_;
    if (sec_check.count() > 1.0) {
        LOG_ERROR("!!!!!!!PCAN WRITE TIMEOUT PROCESS EXIT!!!!!!!");
        std::exit(0);
    }
    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start_time;
    if (sec.count() * 1000 > 3) {
        LOG_ERROR("PCANWRITE: %f ms", sec.count() * 1000);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
}

void Pcan_driver::Read_Loop()
{
    TPCANMsg o_canmsg;
    while (!b_terminate_) {
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        TPCANStatus status = safe_CAN_Read(st_interface_param_.st_pcan_param.n_pcan_bus, &o_canmsg, NULL);
        std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start_time;
        if (status == PCAN_ERROR_OK) {
            Notify("PcanCallback", o_canmsg);
            Notify("pcan_callback", o_canmsg);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        else {  // 피캔 데이터가 없을 때 sleep걸어주기!
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
        if (sec.count() * 1000 > 3) {
            LOG_ERROR("PCANREAD: %f ms", sec.count() * 1000);
        }
    }
}

void Pcan_driver::WriteCan(const int& id, const vector<int>& data)
{
    Write(id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

bool Pcan_driver::Notify(const std::string& str_cbf_name, const boost::any& any_type_var)
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

bool Pcan_driver::RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end()) {
        map_callback_pt_[str_cbf_name] = pt_func;
        return true;
    }
    else {
        return false;
    }
}

MotorState Pcan_driver::handleMotorControl(int motorStatus)
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

    return state;
}
#else  // peak can usb 모듈이 없는 경우
MotorState Pcan_driver::handleMotorControl(int motorStatus)
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

    return state;
}
#endif
