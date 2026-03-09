#include "NaviCAN/MotorDriver/MotorDriver.h"

#include "NaviCAN/MotorDriver/MotorDriverRegistrar.h"
#include "util/logger.hpp"

using namespace NaviFra;
using namespace NaviFra::NaviCAN::Object::Standard;
using namespace NaviFra::NaviCAN::Canopen::CIA402;

namespace detail {

template <uint8_t MultiAxisIndex, uint8_t MaxIndex = 7>
struct IndexSelector {
    // 모터 모드 등록
    template <typename Driver>
    static void registerModes(Driver* driver)
    {
        if (driver->getMultiAxisIndex() == MultiAxisIndex) {
            driver->template registerCommonModes<Modes<MultiAxisIndex>>();
            return;
        }
        if constexpr (MultiAxisIndex < MaxIndex) {
            IndexSelector<MultiAxisIndex + 1, MaxIndex>::registerModes(driver);
        }
    }
};
}  // namespace detail

MotorDriver::MotorDriver(const MotorDriverParams& params)
    : IMotorDriverBase(params)
    , multiaxis_index_(params.multiaxis_index)
    , mode_(params.mode_of_operation)
    , initial_state_(params.initial_state)
    , enable_first_(params.enable_first)
{
    // 인덱스 기반으로 CANOPEN 인덱스 설정
    switch (multiaxis_index_) {
        case 0:
            setupIndex<0>();
            break;
        case 1:
            setupIndex<1>();
            break;
        case 2:
            setupIndex<2>();
            break;
        case 3:
            setupIndex<3>();
            break;
        case 4:
            setupIndex<4>();
            break;
        case 5:
            setupIndex<5>();
            break;
        case 6:
            setupIndex<6>();
            break;
        case 7:
            setupIndex<7>();
            break;
        default:
            setupIndex<0>();
            break;  // 기본값
    }
}

bool MotorDriver::setTarget(double val)
{
    if (state_handler_.getState() == StateHandler::State::OPERATION_ENABLE) {
        std::scoped_lock lock(mode_mutex_);
        return selected_mode_ && selected_mode_->setTarget(val);
    }
    return false;
}

bool MotorDriver::isModeSupported(int8_t mode)
{
    // homing mode 의도적 제외
    return mode != IMotorDriver::HOMING && allocMode(mode);
}

bool MotorDriver::enterModeAndWait(int8_t mode)
{
    // homing mode 의도적 제외
    bool okay = mode != IMotorDriver::HOMING && switchMode(mode);
    return okay;
}

int8_t MotorDriver::getMode()
{
    std::scoped_lock lock(mode_mutex_);
    return selected_mode_ ? selected_mode_->mode_id_ : (int8_t)IMotorDriver::NO_MODE;
}

bool MotorDriver::isModeSupportedByDevice(int8_t mode)
{
    uint32_t supported_modes = getSupportedModes();
    bool supported = supported_modes & (1 << (mode - 1));
    bool below_max = mode <= 32;
    bool above_min = mode > 0;
    return below_max && above_min && supported;
}

void MotorDriver::registerMode(int8_t id, const ModeSharedPtr& m)
{
    std::scoped_lock map_lock(map_mutex_);
    if (m && m->mode_id_ == id)
        modes_.insert(std::make_pair(id, m));
}

ModeSharedPtr MotorDriver::allocMode(int8_t mode)
{
    ModeSharedPtr res;
    // if (isModeSupportedByDevice(mode)) {
    std::scoped_lock map_lock(map_mutex_);
    auto it = modes_.find(mode);

    if (it != modes_.end()) {
        res = it->second;
    }
    // }
    return res;
}

bool MotorDriver::switchMode(int8_t mode)
{
    if (mode == IMotorDriver::NO_MODE) {
        std::scoped_lock lock(mode_mutex_);
        selected_mode_.reset();
        try {
            setOpMode(mode);
        }
        catch (...) {
        }
        return true;
    }

    ModeSharedPtr next_mode = allocMode(mode);
    if (!next_mode) {
        NLOG(info) << "Mode is not supported.";
        return false;
    }

    if (!next_mode->start()) {
        NLOG(info) << "Could not start mode.";
        return false;
    }

    {
        std::scoped_lock lock(mode_mutex_);
        if (mode_id_ == mode && selected_mode_ && selected_mode_->mode_id_ == mode) {
            return true;
        }
        selected_mode_.reset();
    }

    // if (!switchState(switching_state_)) return false;

    setOpMode(mode);

    bool okay = false;

    {
        std::unique_lock lock(mode_mutex_);

        std::chrono::steady_clock::time_point abstime = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        if (monitor_mode_) {
            while (mode_id_ != mode && mode_cond_.wait_until(lock, abstime) == std::cv_status::no_timeout) {}
        }
        else {
            while (mode_id_ != mode && std::chrono::steady_clock::now() < abstime) {
                lock.unlock();

                // driver_->universal_get_value<int8_t>(op_mode_display_index, 0x0);
                // 이부분 동작확인
                mode_id_ = getOpMode();

                std::this_thread::sleep_for(20ms);
                lock.lock();
            }
        }

        if (mode_id_ == mode) {
            NLOG(info) << "switchMode complete mode=" << (int)mode;
            selected_mode_ = next_mode;
            okay = true;
        }
        else {
            NLOG(info) << "Mode switch timed out.";
            setOpMode(mode_id_);
        }
    }

    if (!switchState(StateHandler::State::OPERATION_ENABLE)) {
        return false;
    }

    return okay;
}

bool MotorDriver::switchState(const StateHandler::State& target)
{
    std::chrono::steady_clock::time_point abstime = std::chrono::steady_clock::now() + state_switch_timeout_;
    auto state = state_handler_.getState();
    target_state_ = target;

    while (state != target_state_) {
        std::unique_lock lock(cw_mutex_);
        auto next = StateHandler::UNKNOWN;
        bool success = Command::setTransition(control_word_, state, target_state_, &next);
        lock.unlock();

        if (!success) {
            NLOG(info) << "Could not set transition.";
            return false;
        }

        if (state != next && !state_handler_.waitForNewState(abstime, state)) {
            NLOG(info) << "Transition timed out. motorstate " << std::to_string(state);
            return false;
        }

        std::this_thread::sleep_for(100ms);
    }
    return state == target;
}

bool MotorDriver::readState()
{
    uint16_t sw = getStatusWord();
    uint16_t old_sw = status_word_.exchange(sw);

    if (old_sw != sw) {
        NLOG(info) << "[MotorId " << (int)driver_->id() << "] Status word : " << std::hex << old_sw << " -> " << sw;
    }

    state_handler_.updateState(status_word_);

    std::unique_lock lock(mode_mutex_);
    uint16_t new_mode = getOpMode();

    if (selected_mode_ && selected_mode_->mode_id_ == new_mode) {
        if (!selected_mode_->read(sw)) {
            // NLOG(info) << "Mode handler has error.";
        }
    }
    if (new_mode != mode_id_) {
        mode_id_ = new_mode;
        mode_cond_.notify_all();
    }
    if (selected_mode_ && selected_mode_->mode_id_ != new_mode) {
        // NLOG(info) << "Mode does not match.";
    }

    return true;
}

void MotorDriver::handleRead()
{
    readState();
}

void MotorDriver::handleWrite()
{
    std::scoped_lock lock(cw_mutex_);

    control_word_ |= (1 << ControlWord::Bits::HALT);

    if (state_handler_.getState() == StateHandler::State::OPERATION_ENABLE) {
        std::scoped_lock lock(mode_mutex_);
        uint16_t cw = control_word_.getValue();
        Mode::OpModeAccesser cwa(cw);
        bool okay = false;
        if (selected_mode_ && selected_mode_->mode_id_ == mode_id_) {
            okay = selected_mode_->write(cwa);
        }
        else {
            cwa = 0;
        }
        if (okay) {
            static uint16_t cwa_mask = (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC0) |
                (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC1) | (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC2);
            control_word_ &= ~cwa_mask;
            control_word_ |= (cwa.get() & cwa_mask);
            control_word_ &= ~(1 << ControlWord::Bits::HALT);
        }
    }
    if (start_fault_reset_.exchange(false)) {
        NLOG(info) << "Fault reset";
        control_word_ &= ~(1 << ControlWord::Bits::FAULT_RESET);
    }

    setControlWord(control_word_.getValue());
}

bool MotorDriver::handleInit()
{
    registerDefaultModes();
    startThread();
    std::this_thread::sleep_for(100ms);
    for (auto it = mode_allocators_.begin(); it != mode_allocators_.end(); ++it) {
        (it->second)();
    }

    NLOG(info) << "Init: Read State";
    if (!readState()) {
        NLOG(info) << "Could not read motor state";
        return false;
    }
    {
        std::scoped_lock lock(cw_mutex_);
        control_word_.setValue(0);
        start_fault_reset_ = true;
    }

    control_word_.faultReset(false);
    std::this_thread::sleep_for(200ms);
    control_word_.faultReset(true);
    std::this_thread::sleep_for(400ms);
    control_word_.faultReset(false);
    std::this_thread::sleep_for(200ms);

    int timeout = 30;  // 3 seconds timeout

    while (timeout > 0) {
        if (status_word_.isFault() == false) {
            NLOG(info) << "[handleInit] Fault reset successful.";
            break;
        }
        std::this_thread::sleep_for(100ms);
        timeout -= 1;
    }

    if (timeout <= 0) {
        NLOG(info) << "[handleInit] Fault reset timed out.";
        return false;
    }

    if (enable_first_) {  // set motor enable -> set mode
        NLOG(info) << "Init: Enable. driver_id=" << (int)driver_->id();
        if (!switchState(initial_state_)) {
            NLOG(info) << "Could not enable motor";
            return false;
        }

        enterModeAndWait(mode_);
    }
    else {  // set mode -> set motor enable
        enterModeAndWait(mode_);

        NLOG(info) << "Init: Enable driver_id=" << (int)driver_->id();
        if (!switchState(initial_state_)) {
            NLOG(info) << "Could not enable motor";
            return false;
        }
    }

    return true;
}

bool MotorDriver::handleEnable()
{
    bool ret = switchState(StateHandler::State::OPERATION_ENABLE);
    return ret;
}

bool MotorDriver::handleDisable()
{
    bool ret = switchState(StateHandler::State::SWITCH_ON_DISABLED);

    return ret;
}

bool MotorDriver::handleShutdown()
{
    mode_allocators_.clear();

    bool ret = switchState(StateHandler::State::SWITCH_ON_DISABLED);

    stopThread();

    return ret;
}

bool MotorDriver::handleHalt()
{
    auto state = state_handler_.getState();
    std::scoped_lock lock(cw_mutex_);

    // do not demand quickstop in case of fault
    if (state == StateHandler::State::FAULT_REACTION_ACTIVE || state == StateHandler::State::FAULT)
        return false;

    if (state != StateHandler::State::OPERATION_ENABLE) {
        target_state_ = state;
    }
    else {
        target_state_ = StateHandler::State::QUICK_STOP_ACTIVE;
        if (!Command::setTransition(control_word_, state, StateHandler::State::QUICK_STOP_ACTIVE, 0)) {
            NLOG(info) << "Could not quick stop";
            return false;
        }
    }
    return true;
}

bool MotorDriver::handleRecover()
{
    if (status_word_.isFault() == false) {
        NLOG(info) << "[handleRecover] motor is not in fault state, no need to recover.";
        return false;
    }
    control_word_.faultReset(false);
    std::this_thread::sleep_for(100ms);
    control_word_.faultReset(true);
    std::this_thread::sleep_for(200ms);
    control_word_.faultReset(false);
    std::this_thread::sleep_for(100ms);

    int timeout = 30;  // 3 seconds timeout

    while (timeout > 0) {
        if (status_word_.isFault() == false) {
            NLOG(info) << "[handleRecover] Fault reset successful.";
            break;
        }
        std::this_thread::sleep_for(100ms);
        timeout -= 1;
    }

    if (timeout <= 0) {
        NLOG(info) << "[handleRecover] Fault reset timed out.";
        return false;
    }

    // 모드 변경하는 건 순서 확인
    enterModeAndWait(mode_);

    {
        std::scoped_lock lock(mode_mutex_);
        if (selected_mode_ && !selected_mode_->start()) {
            NLOG(info) << "[handleRecover] Could not restart mode.";
            return false;
        }
    }

    // 리셋 시 딜레이 필요
    if (!switchState(StateHandler::State::SWITCH_ON_DISABLED)) {
        NLOG(info) << "[handleRecover] Could not switch to SWITCH_ON_DISABLED";
        return false;
    }

    // 리셋 시 딜레이 필요
    std::this_thread::sleep_for(500ms);

    if (!switchState(StateHandler::State::OPERATION_ENABLE)) {
        NLOG(info) << "[handleRecover] Could not switch to OPERATION_ENABLE";
        return false;
    }

    return true;
}

bool MotorDriver::handleHoming(int8_t homing_method, std::chrono::milliseconds timeout)
{
    // Homing 모드는 switchMode를 사용하지 않고 직접 할당
    selected_mode_.reset();
    selected_mode_ = allocMode(IMotorDriver::HOMING);
    if (!selected_mode_) {
        NLOG(info) << "Homing mode is not registered or supported";
        return false;
    }

    // HomingMode로 캐스팅
    auto homing_mode = std::dynamic_pointer_cast<HomingMode>(selected_mode_);
    if (!homing_mode) {
        NLOG(info) << "Cannot convert mode to HomingMode";
        return false;
    }

    setOpMode(IMotorDriver::HOMING);
    std::this_thread::sleep_for(200ms);

    if (!switchState(StateHandler::State::SWITCH_ON_DISABLED)) {
        NLOG(info) << "Cannot enable motor for homing";
        return false;
    }

    std::this_thread::sleep_for(1s);
    if (homing_mode->setHomingMethod(homing_method) == false) {
        // 원래 모드로 복귀
        if (mode_ != IMotorDriver::HOMING) {
            enterModeAndWait(mode_);
        }

        // OPERATION_ENABLE 상태
        if (state_handler_.getState() != StateHandler::State::OPERATION_ENABLE) {
            if (!switchState(StateHandler::State::OPERATION_ENABLE)) {
                NLOG(info) << "Cannot enable motor for homing";
                return false;
            }
        }
        return false;
    };

    std::this_thread::sleep_for(200ms);

    // OPERATION_ENABLE 상태
    if (state_handler_.getState() != StateHandler::State::OPERATION_ENABLE) {
        if (!switchState(StateHandler::State::OPERATION_ENABLE)) {
            NLOG(info) << "Cannot enable motor for homing";
            return false;
        }
    }

    // Homing 실행
    bool result = homing_mode->homing(timeout);

    // 원래 모드로 복귀
    if (mode_ != IMotorDriver::HOMING) {
        enterModeAndWait(mode_);
    }

    return result;
}

template <typename T, typename... Args>
bool MotorDriver::registerMode(int8_t mode, Args&&... args)
{
    return mode_allocators_
        .insert(std::make_pair(
            mode,
            [args..., mode, this]() {
                // if (isModeSupportedByDevice(mode))
                registerMode(mode, std::make_shared<T>(args...));
            }))
        .second;
}

template <uint8_t MotorIndex>
void MotorDriver::setupIndex()
{
    status_word_entry_index = CanopenIndex<MotorIndex>::status_word_entry_index;
    control_word_entry_index = CanopenIndex<MotorIndex>::control_word_entry_index;
    op_mode_index = CanopenIndex<MotorIndex>::op_mode_index;
    op_mode_display_index = CanopenIndex<MotorIndex>::op_mode_display_index;
    supported_drive_modes_index = CanopenIndex<MotorIndex>::supported_drive_modes_index;
    velocity_actual_value_index = CanopenIndex<MotorIndex>::velocity_actual_value_index;
    position_actual_value_index = CanopenIndex<MotorIndex>::position_actual_value_index;
    save_parameters_index = CanopenIndex<MotorIndex>::save_parameters_index;
    save_all_parameters_subindex = CanopenIndex<MotorIndex>::save_all_parameters_subindex;
}

void MotorDriver::registerDefaultModes()
{
    detail::IndexSelector<0>::registerModes(this);
}
