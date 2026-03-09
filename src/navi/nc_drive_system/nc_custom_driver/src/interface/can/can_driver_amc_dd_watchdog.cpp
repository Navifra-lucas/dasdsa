#include "core_msgs/MotorServoStatus.h"
#include "interface/can/can_driver_amc_dd.hpp"
#include "util/logger.hpp"

using namespace NaviFra;

void Can_driver_amc_dd::watchdog()
{
    while (running_) {
        /*
            Driver가 초기화 되고 모터가 enable 상태가 아닐 경우 Enable로 변경
            만약 초기화 중이 면 continue;
        */

        auto motor_data = GetMotorData();

        if (b_init_&& !b_brake_on_) {
            for (auto [id, data] : motor_data) {
                if(id == MotorID::FR_TRACTION || id == MotorID::RL_TRACTION)
                    continue;
                // switch on status check
                int motor_id = id;
                // 0x4F = 0100 1111 , bit0: Ready to Switch On, bit1: Switched On, bit2: Operation Enabled, bit3: Fault, bit6: Switch On Disabled
                int n_check_status = data.status & 0x4F; 
                if (n_check_status == STATUS_READY_SWITCH_ON) {
                    LOG_WARNING("motor currunet state STATUS_READY_SWITCH_ON try enablde motor %d", motor_id);
                    sendControlWord(CONTROL_WORD_SWITCH_ON, motor_id);  // ControlWord 사용
                }
                else if (n_check_status == STATUS_SWITCHED_ON) {
                    sendControlWord(CONTROL_WORD_ENABLE_OPERATION, motor_id);  // ControlWord 사용
                }
                else if (n_check_status == STATUS_NOT_READY || n_check_status == STATUS_SWITCH_ON_DISABLED) {
                    LOG_WARNING("motor currunet state STATUS_SWITCH_ON_DISABLED try enablde motor %d", motor_id);
                    sendControlWord(CONTROL_WORD_SHUTDOWN, motor_id);  // ControlWord 사용
                }
                else if (n_check_status == STATUS_FAULT) {
                    LOG_WARNING("motor currunet state STATUS_FAULT try enablde motor %d", motor_id);
                    sendControlWord(CONTROL_WORD_FAULT_RESET, motor_id);  // ControlWord 사용
                }
                
            }
        }

        // CheckError(motor_data);
        Callback(motor_data);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Can_driver_amc_dd::Callback(std::map<int16_t, motor_msgs::MotorData> motor_data)
{
    static std::map<int16_t, int32_t> encoder_data;
    motor_msgs::MotorDataInfo motor_data_info;

    for (auto [id, data] : motor_data) {
        if (data.encoder == 0 && abs(data.encoder - encoder_data[id]) > 10000)
            encoder_data[id] = 0;
        int32_t n_acc = data.encoder - encoder_data[id];
        bool b_error = false;
        if (abs(n_acc) > (16777216 - 5000) && abs(n_acc) < (16777216 + 5000)) {
            b_error = true;
            if (n_acc > 0)
                n_acc -= 16777216;
            else
                n_acc += 16777216;
        }
        if (abs(n_acc) > 1000000) {
            LOG_ERROR("dodom_acc now %d pre %d", data.encoder, encoder_data[id]);
        }
        if (b_error) {
            LOG_WARNING("dodom_warn, now %d pre %d", data.encoder, encoder_data[id]);
        }
        encoder_data[id] = data.encoder;
        // LOG_INFO("Final n_acc for motor_id: %d = %d", id, n_acc);
        motor_msgs::MotorData motordata;
        int n_check_status = data.status & 0x4F;
        if(id == MotorID::FL_TRACTION){
            motordata.motor_id = id;
            motordata.status = data.status;
            motordata.error_code = data.error_code;
            // motordata.is_enable = true;
            if(n_check_status == STATUS_OP_ENABLED)
                motordata.feedback_brake_on = false;
            else
                motordata.feedback_brake_on = true;
            motordata.is_error = data.is_error;
            motordata.current = data.current;
            motordata.input_velocity = data.input_velocity;
            motordata.feedback_velocity = data.feedback_velocity;
            motordata.encoder = data.encoder;
            motordata.acc_encoder = n_acc;
            motordata.is_error = data.is_error;
            motordata.last_update_time = data.last_update_time;
            motor_data_info.data.push_back(motordata);
        }
        else if(id == MotorID::RR_TRACTION){
            motordata.motor_id = id;
            motordata.status = data.status;
            // motordata.is_enable = true;
            if(n_check_status == STATUS_OP_ENABLED)
                motordata.feedback_brake_on = false;
            else
                motordata.feedback_brake_on = true;
            motordata.error_code = data.error_code;
            motordata.is_error = data.is_error;
            motordata.current = data.current;
            motordata.input_velocity = data.input_velocity;
            motordata.feedback_velocity = data.feedback_velocity;
            motordata.encoder = data.encoder;
            motordata.acc_encoder = n_acc;
            motordata.is_error = data.is_error;
            motordata.last_update_time = data.last_update_time;
            motor_data_info.data.push_back(motordata);
        }

    }
    Communicator::Notify("motor_data_callback", motor_data_info);
}

void Can_driver_amc_dd::CheckError(std::map<int16_t, motor_msgs::MotorData> motor_data)
{
    static std::map<int16_t, std::chrono::system_clock::time_point> current_time;
    motor_msgs::MotorDataInfo motor_data_info;

    for (auto [id, data] : motor_data) {
        // error check
        float f_time_thres = 0.4;
        std::chrono::nanoseconds ns_since_epoch(static_cast<long long>(data.last_update_time * 1e9));  // 초를 나노초로 변환
        std::chrono::time_point<std::chrono::system_clock> tp_update(ns_since_epoch);
        std::chrono::duration<double> sec = std::chrono::system_clock::now() - tp_update;
        motor_msgs::MotorData motordata;

        if (sec.count() > f_time_thres) {
            if (id == MotorID::FL_TRACTION) {
                motordata.motor_id = id;
                motordata.is_error = true;
                motordata.error_code = core_msgs::NaviAlarm::ERROR_FL_TRACTION_MOTOR_FEEDBACK_TIMEOUT;
                motordata.error_msg = "ERROR_FL_TRACTION_MOTOR_FEEDBACK_TIMEOUT";
            }
            else if (id == MotorID::RR_TRACTION) {
                motordata.motor_id = id;
                motordata.is_error = true;
                motordata.error_code = core_msgs::NaviAlarm::ERROR_RR_TRACTION_MOTOR_FEEDBACK_TIMEOUT;
                motordata.error_msg = "ERROR_RR_TRACTION_MOTOR_FEEDBACK_TIMEOUT";
            }
        }

        if(data.current < st_driver_param_.f_traction_overcurrent_amp_std)
        {
            if (id == MotorID::FL_TRACTION) {
                current_time[id] = std::chrono::system_clock::now();
            }
            else if (id == MotorID::RR_TRACTION) {
                current_time[id] = std::chrono::system_clock::now();
            }
        }
        std::chrono::duration<double> current_check_time = std::chrono::system_clock::now() - current_time[id];
        if(current_check_time.count() > st_driver_param_.f_traction_overcurrent_time_std)
        {
            if (id == MotorID::FL_TRACTION) {
                motordata.motor_id = id;
                motordata.is_error = true;
                motordata.error_code = core_msgs::NaviAlarm::ERROR_FL_TRACTION_MOTOR_OVER_CURRENT;
                motordata.error_msg = "ERROR_FL_TRACTION_MOTOR_OVER_CURRENT";
            }
            else if (id == MotorID::RR_TRACTION) {
                motordata.motor_id = id;
                motordata.is_error = true;
                motordata.error_code = core_msgs::NaviAlarm::ERROR_RR_TRACTION_MOTOR_OVER_CURRENT;
                motordata.error_msg = "ERROR_RR_TRACTION_MOTOR_OVER_CURRENT";
            }
        }

        static int n_fl_traction_fault_cnt = 0;
        static int n_rr_traction_fault_cnt = 0;
        if (data.is_error == true) {
            if (id == MotorID::FL_TRACTION)
                n_fl_traction_fault_cnt++;
            else if (id == MotorID::RR_TRACTION)
                n_rr_traction_fault_cnt++;            
        }
        else {
            if (id == MotorID::FL_TRACTION)
                n_fl_traction_fault_cnt = 0;
            else if (id == MotorID::RR_TRACTION)
                n_rr_traction_fault_cnt = 0;  
        }

        static double FAULT_RESET_DELAY_SEC = 5.0;

        ostringstream oss_error;
        oss_error << hex << data.error_code;
        int n_error = std::stoi(oss_error.str());

        if (n_fl_traction_fault_cnt > 10) {
            motordata.motor_id = MotorID::FL_TRACTION;
            motordata.is_error = true;
            motordata.error_code = core_msgs::NaviAlarm::ERROR_FL_TRACTION_MOTOR_FAULT;
            motordata.error_msg = "ERROR_FL_TRACTION_MOTOR_FAULT," + oss_error.str();

            std::chrono::duration<double> fl_fault_reset = std::chrono::system_clock::now() - current_time[MotorID::FL_TRACTION];

            if (fl_fault_reset.count() > FAULT_RESET_DELAY_SEC) {
                sendControlWord(CONTROL_WORD_FAULT_RESET, MotorID::FL_TRACTION);
                current_time[MotorID::FL_TRACTION] = std::chrono::system_clock::now();
            }
        }
        else if (n_rr_traction_fault_cnt > 10) {
            motordata.motor_id = MotorID::RR_TRACTION;
            motordata.is_error = true;
            motordata.error_code = core_msgs::NaviAlarm::ERROR_RR_TRACTION_MOTOR_FAULT;
            motordata.error_msg = "ERROR_RR_TRACTION_MOTOR_FAULT," + oss_error.str();

            std::chrono::duration<double> rr_fault_reset = std::chrono::system_clock::now() - current_time[MotorID::RR_TRACTION];

            if (rr_fault_reset.count() > FAULT_RESET_DELAY_SEC) {
                sendControlWord(CONTROL_WORD_FAULT_RESET, MotorID::RR_TRACTION);  // ControlWord 사용
                current_time[MotorID::RR_TRACTION] = std::chrono::system_clock::now();
            }
        }
        motor_data_info.data.push_back(motordata);
    }
    Communicator::Notify("motor_data_callback", motor_data_info);
}