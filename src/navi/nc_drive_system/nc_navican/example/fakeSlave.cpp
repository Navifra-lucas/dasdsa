#include "fakeSlave.h"

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <csignal>
#include <iostream>

using namespace NaviFra;

std::atomic<bool> running(true);

void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running.store(false);
}

void CIA402MockSlave::run_profiled_position_mode()
{
    std::cout << "run_profiled_position_mode" << std::endl;
    double profile_speed = static_cast<double>(((uint32_t)(*this)[0x6081][0])) / 1000;
    double profile_accerlation = static_cast<double>(((uint32_t)(*this)[0x6083][0])) / 1000;
    double actual_position = static_cast<double>(((int32_t)(*this)[0x6064][0])) / 1000.0;
    double target_position = static_cast<double>(((int32_t)(*this)[0x607A][0])) / 1000.0;
    double actual_speed = static_cast<double>(((int32_t)(*this)[0x606C][0])) / 1000.0;
    std::cout << "Profile_Speed " << profile_speed << "Profile Acceleration: " << profile_accerlation << std::endl;

    while ((state.load() == InternalState::Operation_Enable) && (operation_mode.load() == Profiled_Position) /* && running.load()*/) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        target_position = static_cast<double>(((int32_t)(*this)[0x607A][0])) / 1000.0;
        if (target_position != actual_position) {
            clear_status_bit(SW_Operation_mode_specific0);
            clear_status_bit(SW_Target_reached);
            {
                std::scoped_lock<std::mutex> lock(w_mutex);
                (*this)[0x6041][0] = status_word;
                this->TpdoEvent(1);
            }
            is_new_set_point.store(false);
            // RCLCPP_INFO(
            // rclcpp::get_logger("cia402_slave"), "Move from %f to %f", actual_position,
            // target_position);
            {
                MotionGenerator gen(profile_accerlation, profile_speed, actual_position);

                while (!gen.getFinished()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    actual_position = gen.update(target_position);
                    actual_speed = gen.getVelocity();
                    (*this)[0x6064][0] = (int32_t)(actual_position * 1000);
                    (*this)[0x606C][0] = (int32_t)(actual_speed * 1000);
                }
            }

            std::cout << "Reached target position " << actual_position << std::endl;
            clear_status_bit(SW_Operation_mode_specific0);
            set_status_bit(SW_Target_reached);
            {
                std::scoped_lock<std::mutex> lock(w_mutex);
                (*this)[0x6041][0] = status_word;
                this->TpdoEvent(1);
            }
        }
    }
    mode_select_ = false;
}

void CIA402MockSlave::run_cyclic_position_mode()
{
    std::cout << "run_cyclic_position_mode" << std::endl;
    int32_t min_pos = (int32_t)(*this)[0x607D][1];
    int32_t max_pos = (int32_t)(*this)[0x607D][2];
    uint8_t int_period = (*this)[0x60C2][1];
    int32_t offset = (*this)[0x60B0][0];
    int8_t index = (*this)[0x60C2][2];

    std::cout << "Lower Software Limit: " << min_pos << std::endl;
    std::cout << "Upper Software Limit: " << max_pos << std::endl;
    // RCLCPP_INFO(
    //  rclcpp::get_logger("cia402_slave"), "Control Cycle: %hhu + 10^%hhd", int_period, index);
    std::cout << "Offset: " << offset << std::endl;

    double cp_min_position = min_pos / 1000;
    double cp_max_position = max_pos / 1000;
    double cp_interpolation_period = int_period * std::pow(10.0, index);
    double cp_offset = (double)(offset / 1000.0);
    int ccp_millis = (int)(control_cycle_period * std::pow(10.0, 3));
    int32_t act_pos;
    while ((state.load() == InternalState::Operation_Enable) &&
           (operation_mode.load() == Cyclic_Synchronous_Position) /* && running.load()*/) {
        act_pos = (*this)[0x607A][0];
        double target_position = (act_pos) / 1000 - cp_offset;  // m
        double position_delta = target_position - actual_position;  // m
        double speed = position_delta / cp_interpolation_period;  // m/s
        double increment = control_cycle_period * speed;  // m
        (*this)[0x606C][0] = (int32_t)speed * 1000;
        if ((target_position < cp_max_position) && (target_position > cp_min_position) && (std::abs(position_delta) > 0.001)) {
            while ((std::abs(actual_position - target_position) > 0.001) /* && running.load()*/) {
                std::this_thread::sleep_for(std::chrono::milliseconds(ccp_millis));
                actual_position += increment;
                (*this)[0x6064][0] = (int32_t)actual_position * 1000;
                if (std::abs(actual_position - target_position) < 0.001) {
                    std::cout << "Reached Target " << target_position << std::endl;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(ccp_millis));
    }
    mode_select_ = false;
}

void CIA402MockSlave::run_interpolated_position_mode()
{
    std::cout << "run_interpolated_position_mode" << std::endl;
    //  Retrieve parameters from the object dictionary
    double interpolation_period = static_cast<double>((uint8_t)(*this)[0x60C2][1]);
    double target_position = static_cast<double>((int32_t)(*this)[0x60C1][1]);

    // int32_t offset = (*this)[0x60B0][0];

    // Convert parameters to SI units
    interpolation_period *= std::pow(10.0, static_cast<double>((int8_t)(*this)[0x60C2][2]));
    target_position /= 1000.0;
    double actual_position = static_cast<double>((int32_t)(*this)[0x6064][0]) / 1000.0;

    // RCLCPP_INFO(
    //   rclcpp::get_logger("cia402_slave"), "Interpolation Period: %f", interpolation_period);
    std::cout << "Target position: " << target_position << std::endl;

    while ((state.load() == InternalState::Operation_Enable) && (operation_mode.load() == Interpolated_Position) /* && (rclcpp::ok())*/) {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(interpolation_period * 1000)));

        target_position = static_cast<double>((int32_t)(*this)[0x60C1][1]) / 1000.0;

        if (target_position != actual_position) {
            double position_delta = target_position - actual_position;
            double position_increment = position_delta / 20;
            clear_status_bit(SW_Operation_mode_specific0);
            clear_status_bit(SW_Target_reached);
            {
                std::scoped_lock<std::mutex> lock(w_mutex);
                (*this)[0x6041][0] = status_word;
                this->TpdoEvent(1);
            }

            while ((std::abs(actual_position - target_position) > 0.001) /* && running.load()*/) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                actual_position += position_increment;
                (*this)[0x6064][0] = static_cast<int32_t>(actual_position * 1000);
                (*this)[0x606C][0] = static_cast<int32_t>(position_increment * 1000);
            }

            actual_position = target_position;

            // RCLCPP_DEBUG(rclcpp::get_logger("cia402_slave"), "Reached target: %f", actual_position);
            clear_status_bit(SW_Operation_mode_specific0);
            set_status_bit(SW_Target_reached);
            {
                std::lock_guard<std::mutex> lock(w_mutex);
                (*this)[0x6041][0] = status_word;
                this->TpdoEvent(1);
            }
        }
    }
    mode_select_ = false;
}

void CIA402MockSlave::run_profile_velocity_mode()
{
    std::cout << "run_profile_velocity_mode" << std::endl;
    double actual_position = static_cast<double>(((int32_t)(*this)[0x6064][0]));
    double target_velocity = static_cast<double>(((int32_t)(*this)[0x60FF][0]));
    double old_target = target_velocity;
    double control_cycle_period_d = 0.005   ;//earnest
    while ((operation_mode.load() == Profiled_Velocity) /* && running.load()*/) {
        actual_position = static_cast<double>((int32_t)((*this)[0x6064][0]));
        target_velocity = static_cast<double>((int32_t)((*this)[0x60FF][0]));
        if (old_target != target_velocity) {
            old_target = target_velocity;
            // std::cout <<"[" << std::to_string(id()) <<"] New target velocity: " << target_velocity << std::endl;
        }

        (*this)[0x606C][0] = static_cast<int32_t>(target_velocity);
        (*this)[0x6064][0] = static_cast<int32_t>((actual_position + target_velocity * 10000 / 60 * control_cycle_period_d));
        std::this_thread::sleep_for(std::chrono::milliseconds(((int32_t)(control_cycle_period_d * 1000.0))));//earnest
    }
    std::cout << "Leaving run_profile_velocity_mode" << std::endl;
    mode_select_ = false;
}

void CIA402MockSlave::run_homing_mode()
{
    bool homed = false;
    while ((state.load() == InternalState::Operation_Enable) && (operation_mode.load() == Homing) /*&&
           (rclcpp::ok())*/)
    {
        bool start_homing = control_word & CW_Operation_mode_specific0;

        if (start_homing && !homed) {
            set_status_bit(SW_Manufacturer_specific1);  // Motor active
        }
        else {
            homed = false;
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(w_mutex);
            (*this)[0x6041][0] = status_word;
            this->TpdoEvent(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (start_homing) {
            clear_status_bit(SW_Manufacturer_specific1);  // Motor inactive
            set_status_bit(SW_Target_reached);  // Homing complete
            set_status_bit(SW_Operation_mode_specific0);  // Homing attained
            (*this)[0x6041][0] = status_word;
            (*this)[0x6064][0] = static_cast<int32_t>(0);
            (*this)[0x606C][0] = static_cast<int32_t>(0);
            homed = true;
        }
    }
    mode_select_ = false;
}

void CIA402MockSlave::set_new_status_word_and_state()
{
    switch (state.load()) {
        case InternalState::Not_Ready_To_Switch_On:
            on_not_ready_to_switch_on();
            break;
        case InternalState::Switch_On_Disabled:
            on_switch_on_disabled();
            break;
        case InternalState::Ready_To_Switch_On:
            on_ready_to_switch_on();
            break;
        case InternalState::Switched_On:
            on_switched_on();
            break;
        case InternalState::Operation_Enable:
            on_operation_enabled();
            break;
        case InternalState::Quick_Stop_Active:
            on_quickstop_active();
            break;
        case InternalState::Fault_Reaction_Active:
            break;
        case InternalState::Fault:
            std::cout << "Fault" << std::endl;
            break;
        default:
            break;
    }
}

void CIA402MockSlave::set_status_bit(int bit)
{
    std::scoped_lock<std::mutex> lock(w_mutex);
    status_word |= 1UL << bit;
}

void CIA402MockSlave::clear_status_bit(int bit)
{
    std::scoped_lock<std::mutex> lock(w_mutex);
    status_word &= ~(1UL << bit);
}

void CIA402MockSlave::set_switch_on_disabled()
{
    std::cout << "Switch_On_Disabled" << std::endl;
    state.store(InternalState::Switch_On_Disabled);
    clear_status_bit(SW_Ready_To_Switch_On);
    clear_status_bit(SW_Switched_On);
    clear_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    set_status_bit(SW_Switch_on_disabled);
}

void CIA402MockSlave::set_ready_to_switch_on()
{
    std::cout << "Ready_To_Switch_On" << std::endl;
    state.store(InternalState::Ready_To_Switch_On);
    set_status_bit(SW_Ready_To_Switch_On);
    clear_status_bit(SW_Switched_On);
    clear_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    set_status_bit(SW_Quick_stop);
    clear_status_bit(SW_Switch_on_disabled);
}

void CIA402MockSlave::set_switch_on()
{
    std::cout << "Switched_On" << std::endl;
    state.store(InternalState::Switched_On);
    set_status_bit(SW_Ready_To_Switch_On);
    set_status_bit(SW_Switched_On);
    clear_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    set_status_bit(SW_Quick_stop);
    clear_status_bit(SW_Switch_on_disabled);
}

void CIA402MockSlave::set_operation_enabled()
{
    std::cout << "Operation_Enable" << std::endl;
    state.store(InternalState::Operation_Enable);
    set_status_bit(SW_Ready_To_Switch_On);
    set_status_bit(SW_Switched_On);
    set_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    set_status_bit(SW_Quick_stop);
    clear_status_bit(SW_Switch_on_disabled);
}

void CIA402MockSlave::set_quick_stop()
{
    std::cout << "Quick_Stop_Active" << std::endl;
    state.store(InternalState::Quick_Stop_Active);
    set_status_bit(SW_Ready_To_Switch_On);
    set_status_bit(SW_Switched_On);
    set_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    clear_status_bit(SW_Quick_stop);
    clear_status_bit(SW_Switch_on_disabled);
}

void CIA402MockSlave::on_not_ready_to_switch_on()
{
    set_switch_on_disabled();
}

void CIA402MockSlave::on_switch_on_disabled()
{
    if (is_shutdown()) {
        set_ready_to_switch_on();
    }
}

void CIA402MockSlave::on_ready_to_switch_on()
{
    if (is_disable_voltage()) {
        set_switch_on_disabled();
    }
    if (is_switch_on()) {
        set_switch_on();
    }
    if (is_faul_reset()) {
        set_ready_to_switch_on();
    }
}

void CIA402MockSlave::on_switched_on()
{
    if (is_disable_voltage()) {
        set_switch_on_disabled();
    }
    if (is_shutdown()) {
        set_ready_to_switch_on();
    }
    if (is_enable_operation()) {
        set_operation_enabled();
    }
}

void CIA402MockSlave::on_operation_enabled()
{
    if (is_disable_voltage()) {
        set_switch_on_disabled();
    }
    if (is_shutdown()) {
        set_ready_to_switch_on();
    }
    if (is_switch_on()) {
        set_switch_on();
    }
    if (is_quickstop()) {
        set_quick_stop();
    }
    {
        std::scoped_lock<std::mutex> lock(w_mutex);
        is_relative.store(((control_word >> 6) & 1U) == 1U);
        is_halt.store(((control_word >> 8) & 1U) == 1U);
        is_new_set_point.store(((control_word >> 4) & 1U) == 1U);
    }

    if (old_operation_mode.load() != operation_mode.load()) {
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
          std::cout << "Joined homing_mode thread." << std::endl;
          homing_mode.join();
        }
        if (profiled_velocity_mode.joinable()) {
          std::cout << "Joined profiled_velocity_mode thread." << std::endl;
          profiled_velocity_mode.join();
        }
        old_operation_mode.store(operation_mode.load());
        mode_select_ = true;
        switch (operation_mode.load()) {
            case Cyclic_Synchronous_Position:
                start_sync_pos_mode();
                break;
            case Profiled_Position:
                start_profile_pos_mode();
                break;
            case Interpolated_Position:
                start_interpolated_pos_mode();
                break;
            case Homing:
                start_homing_mode();
                break;
            case Profiled_Velocity:
                start_profile_velocity_mode();
                break;
            default:
                mode_select_ = false;
                break;
        }
    }
    else if(!mode_select_)
    {
        mode_select_ = true;
        switch (operation_mode.load()) {
            case Cyclic_Synchronous_Position:
                start_sync_pos_mode();
                break;
            case Profiled_Position:
                start_profile_pos_mode();
                break;
            case Interpolated_Position:
                start_interpolated_pos_mode();
                break;
            case Homing:
                start_homing_mode();
                break;
            case Profiled_Velocity:
                start_profile_velocity_mode();
                break;
            default:
                mode_select_ = false;
                break;
        }
    }
}

void CIA402MockSlave::start_sync_pos_mode()
{
    cyclic_position_mode = std::thread(std::bind(&CIA402MockSlave::run_cyclic_position_mode, this));
}

void CIA402MockSlave::start_profile_pos_mode()
{
    profiled_position_mode = std::thread(std::bind(&CIA402MockSlave::run_profiled_position_mode, this));
}

void CIA402MockSlave::start_interpolated_pos_mode()
{
    interpolated_position_mode = std::thread(std::bind(&CIA402MockSlave::run_interpolated_position_mode, this));
}

void CIA402MockSlave::start_homing_mode()
{
    homing_mode = std::thread(std::bind(&CIA402MockSlave::run_homing_mode, this));
}

void CIA402MockSlave::start_profile_velocity_mode()
{
    profiled_velocity_mode = std::thread(std::bind(&CIA402MockSlave::run_profile_velocity_mode, this));
}

void CIA402MockSlave::on_quickstop_active()
{
    if (is_enable_operation()) {
        set_operation_enabled();
    }
    if (is_disable_voltage()) {
        set_switch_on_disabled();
    }
}

bool CIA402MockSlave::is_shutdown()
{
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool qs_set = ((control_word >> CW_Quick_Stop) & 1U) == 1U;
    bool ev_set = ((control_word >> CW_Enable_Voltage) & 1U) == 1U;
    bool so_unset = ((control_word >> CW_Switch_On) & 1U) == 0U;

    if (fr_unset && qs_set && ev_set && so_unset) {
        std::cout << "Received Shutdown." << std::endl;
        return true;
    }
    return false;
}

bool CIA402MockSlave::is_disable_voltage()
{
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool ev_unset = ((control_word >> CW_Enable_Voltage) & 1U) == 0U;

    if (fr_unset && ev_unset) {
        std::cout << "Received Disable Voltage." << std::endl;
        return true;
    }
    return false;
}

bool CIA402MockSlave::is_switch_on()
{
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool eo_unset = ((control_word >> CW_Enable_Operation) & 1U) == 0U;
    bool qs_set = ((control_word >> CW_Quick_Stop) & 1U) == 1U;
    bool ev_set = ((control_word >> CW_Enable_Voltage) & 1U) == 1U;
    bool so_set = ((control_word >> CW_Switch_On) & 1U) == 1U;
    if (fr_unset && eo_unset && qs_set && ev_set && so_set) {
        std::cout << "Received Switch On." << std::endl;
        return true;
    }
    return false;
}

bool CIA402MockSlave::is_enable_operation()
{
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool eo_set = ((control_word >> CW_Enable_Operation) & 1U) == 1U;
    bool qs_set = ((control_word >> CW_Quick_Stop) & 1U) == 1U;
    bool ev_set = ((control_word >> CW_Enable_Voltage) & 1U) == 1U;
    bool so_set = ((control_word >> CW_Switch_On) & 1U) == 1U;
    if (fr_unset && eo_set && qs_set && ev_set && so_set) {
        std::cout << "Received Enable Operation." << std::endl;
        return true;
    }
    return false;
}

bool CIA402MockSlave::is_quickstop()
{
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool qs_unset = ((control_word >> CW_Quick_Stop) & 1U) == 0U;
    bool ev_set = ((control_word >> CW_Enable_Voltage) & 1U) == 1U;
    if (fr_unset && qs_unset && ev_set) {
        std::cout << "Received Quick Stop." << std::endl;
        return true;
    }
    return false;
}

bool CIA402MockSlave::is_faul_reset()
{
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_set = ((control_word >> CW_Fault_Reset) & 1U) == 1U;
    if (fr_set) {
        std::cout << "Received Fault Reset." << std::endl;
        return true;
    }
    return false;
}

// This function gets called every time a value is written to the local object
// dictionary by an SDO or RPDO.
void CIA402MockSlave::OnWrite(uint16_t idx, uint8_t subidx) noexcept
{
    // System State
    if (idx == 0x6040 && subidx == 0) {
        {
            std::scoped_lock<std::mutex> lock(w_mutex);
            control_word = (*this)[0x6040][0];
        }
        set_new_status_word_and_state();
        {
            std::scoped_lock<std::mutex> lock(w_mutex);
            (*this)[0x6041][0] = status_word;
            this->TpdoEvent(1);
        }
    }
    // Operation Mode
    if (idx == 0x6060 && subidx == 0) {
        int8_t mode = (*this)[0x6060][0];
        switch (mode) {
            case No_Mode:
            case Profiled_Position:
            case Velocity:
            case Profiled_Velocity:
            case Profiled_Torque:
            case Reserved:
            case Homing:
            case Interpolated_Position:
            case Cyclic_Synchronous_Position:
            case Cyclic_Synchronous_Velocity:
            case Cyclic_Synchronous_Torque:
                operation_mode.store(mode);
                break;
            default:
                std::cout << "Error: Master tried to set unknown operation mode." << std::endl;
        }
        // std::cout << "Switched to mode " << mode << std::endl;

        (*this)[0x6061][0] = (int8_t)(mode);
        this->TpdoEvent(1);
    }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_slave", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // 슬레이브, 타이머, 채널을 모두 저장할 벡터
    std::vector<std::unique_ptr<NaviFra::CIA402MockSlave>> slaves;
    std::vector<std::unique_ptr<io::Timer>> timers;
    std::vector<std::unique_ptr<io::CanChannel>> channels;

    XmlRpc::XmlRpcValue configs;
    try {
        signalHandler(SIGINT);
        signalHandler(SIGTERM);

        io::IoGuard io_guard;
        io::Context ctx;
        io::Poll poll(ctx);
        ev::Loop loop(poll.get_poll());
        auto exec = loop.get_executor();
        
        // 공통으로 사용할 CAN 컨트롤러
        io::CanController ctrl("can0");

        auto sigset_ = lely::io::SignalSet(poll, exec);
        sigset_.insert(SIGHUP);
        sigset_.insert(SIGINT);
        sigset_.insert(SIGTERM);

        sigset_.submit_wait([&](int /*signo*/) {
            sigset_.clear();
            ctx.shutdown();
        });
        std::string navican_path;
        nh.param<std::string>("navican_path", navican_path, "");
        navican_path += "/config/";
        if (ros::param::get("~configs", configs)) {
            std::cout << "Loaded configs: " << configs << std::endl;

            for (int i = 0; i < configs.size(); i++) {
                int node_id = static_cast<int>(configs[i]["node_id"]);
                std::string eds_path = static_cast<std::string>(configs[i]["eds_path"]);
                std::string binary_path = static_cast<std::string>(configs[i]["binary_path"]);
                
                std::cout << "########Node ID: " << node_id << std::endl;
                std::cout << "########EDS Path: " << navican_path + eds_path << std::endl;
                std::cout << "########Binary Path: " << navican_path + binary_path << std::endl;
                
                // 각 슬레이브마다 독립적인 타이머 생성
                timers.push_back(std::make_unique<io::Timer>(poll, exec, CLOCK_MONOTONIC));
                
                // 각 슬레이브마다 독립적인 채널 생성
                channels.push_back(std::make_unique<io::CanChannel>(poll, exec));
                channels.back()->open(ctrl);
                
                // 슬레이브 생성 - 각자의 타이머와 채널 사용
                slaves.emplace_back(std::make_unique<NaviFra::CIA402MockSlave>(
                    *timers.back(), *channels.back(), (navican_path+eds_path).c_str(), (navican_path +binary_path).c_str(), 
                    static_cast<uint8_t>(node_id)));
        
                std::cout << "Created cia402 slave for node_id " << node_id << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
            for (auto &slave : slaves) {
                slave->Reset();
            }
        }
        
        loop.run();
        ctx.shutdown();
        std::cout << "Stopped CANopen Event Loop." << std::endl;
    }
    catch (std::exception& ex) {
        std::cout << "Exception: " << ex.what() << " at line: " << __LINE__ << std::endl;
    }

    return 0;
}