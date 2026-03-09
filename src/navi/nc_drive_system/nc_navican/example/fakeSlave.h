#ifndef NAVIFRA_CIA402_FAKE_SLAVE_HPP
#define NAVIFRA_CIA402_FAKE_SLAVE_HPP
#include "motion_generator.hpp"
#include "state.h"

#include <lely/coapp/sdo.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/ev/co_task.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>

using namespace lely;
using namespace std::chrono_literals;

namespace NaviFra {

class CIA402MockSlave : public canopen::BasicSlave {
public:
    explicit CIA402MockSlave(
        io::TimerBase& timer, io::CanChannelBase& chan, const ::std::string& dcf_txt, const ::std::string& dcf_bin = "", uint8_t id = 0xff)
        : canopen::BasicSlave(timer, chan, dcf_txt, dcf_bin, id)
    {
        state.store(InternalState::Not_Ready_To_Switch_On);
        status_word = 0x0;
        control_word = 0x0;
        operation_mode.store(No_Mode);
        control_cycle_period = 0.01;
        actual_position = 0.0;
    }

    virtual ~CIA402MockSlave()
    {
        if (profiled_position_mode.joinable()) {
            std::cout << "Joined profiled_position_mode thread." << std::endl;
            profiled_position_mode.join();
        }
        if (cyclic_position_mode.joinable()) {
            std::cout << "Joined cyclic_position_mode thread." << std::endl;
            cyclic_position_mode.join();
        }
        if (interpolated_position_mode.joinable()) {
            std::cout << "Joined interpolated_position_mode thread." << std::endl;
            interpolated_position_mode.join();
        }
        if (homing_mode.joinable()) {
            std::cout << "Joined interpolated_position_mode thread." << std::endl;
            homing_mode.join();
        }
        if (profiled_velocity_mode.joinable()) {
            std::cout << "Joined interpolated_position_mode thread." << std::endl;
            profiled_velocity_mode.join();
        }
    }

protected:
    enum InternalState
    {
        Unknown = 0,
        Start = 0,
        Not_Ready_To_Switch_On = 1,
        Switch_On_Disabled = 2,
        Ready_To_Switch_On = 3,
        Switched_On = 4,
        Operation_Enable = 5,
        Quick_Stop_Active = 6,
        Fault_Reaction_Active = 7,
        Fault = 8,
    };

    enum StatusWord
    {
        SW_Ready_To_Switch_On = 0,
        SW_Switched_On = 1,
        SW_Operation_enabled = 2,
        SW_Fault = 3,
        SW_Voltage_enabled = 4,
        SW_Quick_stop = 5,
        SW_Switch_on_disabled = 6,
        SW_Warning = 7,
        SW_Manufacturer_specific0 = 8,
        SW_Remote = 9,
        SW_Target_reached = 10,
        SW_Internal_limit = 11,
        SW_Operation_mode_specific0 = 12,
        SW_Operation_mode_specific1 = 13,
        SW_Manufacturer_specific1 = 14,
        SW_Manufacturer_specific2 = 15
    };
    enum ControlWord
    {
        CW_Switch_On = 0,
        CW_Enable_Voltage = 1,
        CW_Quick_Stop = 2,
        CW_Enable_Operation = 3,
        CW_Operation_mode_specific0 = 4,
        CW_Operation_mode_specific1 = 5,
        CW_Operation_mode_specific2 = 6,
        CW_Fault_Reset = 7,
        CW_Halt = 8,
        CW_Operation_mode_specific3 = 9,
        // CW_Reserved1=10,
        CW_Manufacturer_specific0 = 11,
        CW_Manufacturer_specific1 = 12,
        CW_Manufacturer_specific2 = 13,
        CW_Manufacturer_specific3 = 14,
        CW_Manufacturer_specific4 = 15,
    };

    enum OperationMode
    {
        No_Mode = 0,
        Profiled_Position = 1,
        Velocity = 2,
        Profiled_Velocity = 3,
        Profiled_Torque = 4,
        Reserved = 5,
        Homing = 6,
        Interpolated_Position = 7,
        Cyclic_Synchronous_Position = 8,
        Cyclic_Synchronous_Velocity = 9,
        Cyclic_Synchronous_Torque = 10,
    };
    std::atomic<bool> is_relative;
    std::atomic<bool> is_running;
    std::atomic<bool> is_halt;
    std::atomic<bool> is_new_set_point;
    std::atomic<int8_t> operation_mode;
    std::atomic<int8_t> old_operation_mode;

    std::mutex w_mutex;
    uint16_t status_word;
    uint16_t control_word;
    std::atomic<InternalState> state;

    std::thread profiled_position_mode;
    std::thread profiled_velocity_mode;
    std::thread cyclic_position_mode;
    std::thread cyclic_velocity_mode;
    std::thread interpolated_position_mode;
    std::thread homing_mode;

    double cycle_time;
    bool mode_select_;
    std::mutex in_mode_mutex;
    double actual_position;
    double actual_speed;
    double acceleration;
    double control_cycle_period;

    void run_profiled_position_mode();

    void run_cyclic_position_mode();

    void run_interpolated_position_mode();

    void run_profile_velocity_mode();

    void run_homing_mode();

    void set_new_status_word_and_state();

    void set_status_bit(int bit);

    void set_switch_on_disabled();

    void set_ready_to_switch_on();

    void set_switch_on();

    void set_operation_enabled();

    void set_quick_stop();

    void clear_status_bit(int bit);

    void on_not_ready_to_switch_on();

    void on_switch_on_disabled();

    void on_ready_to_switch_on();

    void on_switched_on();

    void on_operation_enabled();

    void start_sync_pos_mode();

    void start_profile_pos_mode();

    void start_interpolated_pos_mode();

    void start_homing_mode();

    void start_profile_velocity_mode();

    void on_quickstop_active();

    bool is_shutdown();

    bool is_disable_voltage();

    bool is_switch_on();

    bool is_enable_operation();

    bool is_quickstop();

    bool is_faul_reset();

    // This function gets called every time a value is written to the local object
    // dictionary by an SDO or RPDO.
    void OnWrite(uint16_t idx, uint8_t subidx) noexcept override;
};
}  // namespace NaviFra
#endif
