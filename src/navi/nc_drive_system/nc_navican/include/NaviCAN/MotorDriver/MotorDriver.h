#ifndef NAVIFRA_NAVICAN_MOTOR_DRIVER_H
#define NAVIFRA_NAVICAN_MOTOR_DRIVER_H

#include "NaviCAN/MotorDriver/BaseMotorDriver.h"
#include "NaviCAN/MotorDriver/MotorDriverFactory.h"
#include "NaviCAN/MotorDriver/interface/IMotorDriverBase.h"
#include "NaviCAN/MotorDriver/interface/IMotorInfo.h"
#include "NaviCAN/NaviCANDriver.h"
#include "NaviCAN/Object/standard/ControlWord.hpp"
#include "NaviCAN/Object/standard/StatusWord.hpp"
#include "NaviCAN/Utils/CANopenIndexCalculator.h"
#include "NaviCAN/canopen/cia402/Command.h"
#include "NaviCAN/canopen/cia402/DefaultHomingMode.h"
#include "NaviCAN/canopen/cia402/DefaultProfiledPositionMode.h"
#include "NaviCAN/canopen/cia402/ModeForwardHelper.h"
#include "NaviCAN/canopen/cia402/StateHandler.h"
#include "NaviCAN/canopen/cia402/interface/IMotorDriver.h"

#include <algorithm>
#include <atomic>
#include <bitset>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

using namespace NaviFra::NaviCAN::Canopen::CIA402;

namespace NaviFra {

// 채널별로 오프셋 (CANopenIndexCalculator 사용)
template <uint8_t Index>
struct CanopenIndex {
    using Calc = NaviCAN::Utils::CANopenIndexCalculator;
    using Offsets = Calc::CIA402Offsets;

    static constexpr uint16_t base = Calc::getBase(Index);

    //  베이스 있음
    static constexpr uint16_t status_word_entry_index = Calc::getStatusWordIndex(Index);
    static constexpr uint16_t control_word_entry_index = Calc::getControlWordIndex(Index);
    static constexpr uint16_t op_mode_index = Calc::getOperationModeIndex(Index);
    static constexpr uint16_t op_mode_display_index = Calc::getOperationModeDisplayIndex(Index);
    static constexpr uint32_t velocity_actual_value_index = Calc::getVelocityActualValueIndex(Index);
    static constexpr uint32_t position_actual_value_index = Calc::getPositionActualValueIndex(Index);

    //  베이스 없음 (전역 인덱스)
    static constexpr uint16_t supported_drive_modes_index = 0x0502;

    static constexpr uint16_t save_parameters_index = 0x1010;
    static constexpr uint8_t save_all_parameters_subindex = 0x01;
};

template <uint8_t Index>
struct ModesStruct {
    using Calc = NaviCAN::Utils::CANopenIndexCalculator;
    using Offsets = Calc::CIA402Offsets;
    static constexpr uint16_t base = CanopenIndex<Index>::base;

    using ProfiledVelocityMode =
        ModeForwardHelper<IMotorDriver::PROFILED_VELOCITY, int32_t, Calc::calculate(Index, Offsets::TARGET_VELOCITY), 0, 0>;

    using ProfiledTorqueMode =
        ModeForwardHelper<IMotorDriver::PROFILED_TORQUE, int16_t, Calc::calculate(Index, Offsets::TORQUE_TARGET), 0, 0>;

    using CyclicSynchronousPositionMode =
        ModeForwardHelper<IMotorDriver::CYCLIC_SYNCHRONOUS_POSITION, int32_t, Calc::calculate(Index, Offsets::TARGET_POSITION), 0, 0>;

    using CyclicSynchronousVelocityMode =
        ModeForwardHelper<IMotorDriver::CYCLIC_SYNCHRONOUS_VELOCITY, int32_t, Calc::calculate(Index, Offsets::TARGET_VELOCITY), 0, 0>;

    using CyclicSynchronousTorqueMode =
        ModeForwardHelper<IMotorDriver::CYCLIC_SYNCHRONOUS_TORQUE, int16_t, Calc::calculate(Index, Offsets::TORQUE_TARGET), 0, 0>;

    using VelocityMode = ModeForwardHelper<
        IMotorDriver::VELOCITY, int16_t, Calc::calculate(Index, Offsets::VL_TARGET_VELOCITY), 0,
        (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC0) | (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC1) |
            (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC2)>;

    using InterpolatedPositionMode = ModeForwardHelper<
        IMotorDriver::INTERPOLATED_POSITION, int32_t, Calc::calculate(Index, Offsets::INTERPOLATION_DATA_RECORD), 0x01,
        (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC0)>;

    using ProfiledPositionMode = DefaultProfiledPositionMode<
        IMotorDriver::PROFILED_POSITION, int32_t, Calc::calculate(Index, Offsets::TARGET_POSITION), 0,
        Calc::calculate(Index, Offsets::POSITION_ACTUAL_VALUE), 0, 0>;
    using HomingMode = DefaultHomingMode<IMotorDriver::HOMING, int8_t, Calc::calculate(Index, Offsets::HOMING_METHOD), 0, 0>;
};

template <uint8_t Index>
using Modes = ModesStruct<Index>;

// 파라미터
struct MotorDriverParams : public NaviFra::BaseMotorDriverParams {
    StateHandler::State initial_state;
    uint8_t multiaxis_index;
    int8_t mode_of_operation;
    bool enable_first;
};
class MotorDriver
    : public NaviFra::IMotorDriverBase
    , public NaviFra::IMotorDriver {
public:
    using ControlWord = NaviFra::NaviCAN::Object::Standard::ControlWord;
    using StatusWord = NaviFra::NaviCAN::Object::Standard::StatusWord;

    MotorDriver(const MotorDriverParams& params);

    // IMotorDriver 인터페이스 함수
    bool setTarget(double val) override;
    bool enterModeAndWait(int8_t mode) override;
    bool isModeSupported(int8_t mode) override;
    int8_t getMode() override;
    void registerDefaultModes() override;

    // IMotorStateProcessor 인터페이스 함수
    bool handleInit() override;
    void handleRead() override;
    void handleWrite() override;
    bool handleEnable() override;
    bool handleDisable() override;
    bool handleShutdown() override;
    bool handleHalt() override;
    bool handleRecover() override;
    bool handleHoming(int8_t homing_method, std::chrono::milliseconds timeout) override;

    // IMotorInfo 인터페이스
    bool setTargetInfo(double target) override { return setTarget(target); }
    double getSpeed(void) override { return static_cast<double>(getValue<int32_t>(velocity_actual_value_index, 0x00)); }
    double getPosition(void) override { return static_cast<double>(getValue<int32_t>(position_actual_value_index, 0x00)); }
    void preset(void) override {}
    int8_t getOperationMode(void) override { return getValue<int8_t>(op_mode_display_index, 0x00); }
    bool isEnable() override { return state_handler_.getState() == StateHandler::State::OPERATION_ENABLE; }
    bool isFault() override { return state_handler_.getState() == StateHandler::State::FAULT; }
    int getState() override { return state_handler_.getState(); }
    std::string getStateText() override { return state_handler_.getStateText(); }
    uint16_t getStatus() { return getStatusWord(); }

    // 인덱스 (템플릿 메타함수용)
    auto getMultiAxisIndex() const { return multiaxis_index_; }

    // 모터 모드 등록 함수
    template <typename Modes>
    void registerCommonModes()
    {
        // 기본 동작 4 가지
        registerMode<typename Modes::ProfiledVelocityMode>(NaviFra::IMotorDriver::PROFILED_VELOCITY, driver_);
        registerMode<typename Modes::ProfiledPositionMode>(NaviFra::IMotorDriver::PROFILED_POSITION, driver_);
        registerMode<typename Modes::ProfiledTorqueMode>(NaviFra::IMotorDriver::PROFILED_TORQUE, driver_);
        registerMode<typename Modes::HomingMode>(NaviFra::IMotorDriver::HOMING, driver_);
    }

private:
    const uint8_t multiaxis_index_;

    bool isModeSupportedByDevice(int8_t mode);
    void registerMode(int8_t id, const ModeSharedPtr& m);
    ModeSharedPtr allocMode(int8_t mode);
    bool switchMode(int8_t mode);
    bool switchState(const StateHandler::State& target);

    StateHandler state_handler_;

    StatusWord status_word_;
    ControlWord control_word_;
    std::mutex cw_mutex_;

    std::atomic<bool> start_fault_reset_ = false;
    std::atomic<StateHandler::State> target_state_ = StateHandler::State::UNKNOWN;

    std::mutex map_mutex_;
    std::unordered_map<uint16_t, ModeSharedPtr> modes_;
    typedef std::function<void()> AllocFuncType;
    std::unordered_map<uint16_t, AllocFuncType> mode_allocators_;

    ModeSharedPtr selected_mode_;

    int8_t mode_id_ = IMotorDriver::NO_MODE;
    std::condition_variable mode_cond_;
    std::mutex mode_mutex_;

    const bool monitor_mode_ = true;
    const std::chrono::seconds state_switch_timeout_{10};

    const StateHandler::State initial_state_;
    // mode of operation
    const int8_t mode_;
    const bool enable_first_ = true;

    //  canopen object index
    uint16_t status_word_entry_index;
    uint16_t control_word_entry_index;
    uint16_t op_mode_index;
    uint16_t op_mode_display_index;
    uint16_t supported_drive_modes_index;
    uint32_t velocity_actual_value_index;
    uint32_t position_actual_value_index;
    int16_t actual_current_index;
    uint32_t actual_voltage_index;
    uint16_t error_code_index;
    uint16_t save_parameters_index;
    uint8_t save_all_parameters_subindex;

    template <typename T, typename... Args>
    bool registerMode(int8_t mode, Args&&... args);

    template <uint8_t MotorIndex>
    void setupIndex();

    auto getSupportedModes() { return getValue<uint32_t>(supported_drive_modes_index, 0x00); }
    auto getOpMode() { return getValue<int8_t>(op_mode_display_index, 0x00); }
    void setOpMode(int8_t mode) { setValue<int8_t>(op_mode_index, 0x00, mode); }
    uint16_t getStatusWord() { return getValue<uint16_t>(status_word_entry_index, 0x00); }

    void setControlWord(uint16_t value) { setValue<uint16_t>(control_word_entry_index, 0x00, value); }
    void setControlWord(ControlWord& cw) { setValue<uint16_t>(control_word_entry_index, 0x00, cw.getValue()); }

    void saveAllParameters() { setValue<uint32_t>(save_parameters_index, save_all_parameters_subindex, 0x65766173); }

    // 모터 상태 관리 함수
    bool readState();
};

}  // namespace NaviFra

#endif
