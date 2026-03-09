// #include "interface/pcan/pcan_driver_curtis_3d_sd.hpp"

// #include "util/logger.hpp"

// using namespace NaviFra;

// void Pcan_driver_curtis_3d_sd::Initialize()
// {
//     traction_time_ = std::chrono::system_clock::now();
//     steer_time_ = std::chrono::system_clock::now();
//     Pcan_driver::RegisteCallbackFunc("PcanCallback", std::bind(&Pcan_driver_curtis_3d_sd::PcanCallback, this, std::placeholders::_1));
// }

// void Pcan_driver_curtis_3d_sd::EncoderZero(string& str_data)
// {
//     LOG_INFO("SetEncoderZero");
//     Pcan_driver::Write(
//         0x600 + (uint8_t)st_interface_param_.st_pcan_param.n_FL_abssteer_pcan_id, 0x22, 0x03, 0x60, 0x00, 0x00, 0x00, 0x00,
//         0x00);
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     Pcan_driver::Write(
//         0x600 + (uint8_t)st_interface_param_.st_pcan_param.n_FL_abssteer_pcan_id, 0x22, 0x10, 0x10, 0x01, 0x73, 0x61, 0x76,
//         0x65);
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
// }

// void Pcan_driver_curtis_3d_sd::ReInitializeCheck()
// {
//     std::chrono::duration<double> sec_traction_duration = (std::chrono::system_clock::now() - traction_time_);
//     std::chrono::duration<double> sec_steer_duration = (std::chrono::system_clock::now() - steer_time_);
//     std::chrono::duration<double> sec_abs_duration = (std::chrono::system_clock::now() - tp_absencoder_);

//     float f_time_thres = 0.5;
//     if (sec_traction_duration.count() > f_time_thres) {
//         LOG_ERROR("no front traction motor %.3f", sec_traction_duration);
//         LOG_INFO("driver_reset");

//         Pcan_driver::uninit();
//         Pcan_driver::InterfaceOpen();
//         traction_time_ = std::chrono::system_clock::now();
//     }
//     if (sec_steer_duration.count() > f_time_thres) {
//         LOG_ERROR("no front steering motor %.3f", sec_steer_duration);
//         steer_time_ = std::chrono::system_clock::now();
//     }

//     if (sec_abs_duration.count() > f_time_thres) {
//         LOG_ERROR("no abs encoder %.3f", sec_abs_duration);
//         ReInitialize(st_interface_param_.st_pcan_param.n_FL_abssteer_pcan_id);
//         tp_absencoder_ = std::chrono::system_clock::now();
//     }
// }

// void Pcan_driver_curtis_3d_sd::ReInitialize(int n_node_id)
// {
//     std::chrono::duration<double> sec_init = std::chrono::system_clock::now() - tp_reinit_;
//     if (sec_init.count() < 3 || n_FL_traction_feedback_rpm_ != 0) {
//         return;
//     }
//     tp_reinit_ = std::chrono::system_clock::now();
//     LOG_INFO("n_node_id %d ", n_node_id);

//     TPCANMsg o_msg;
//     o_msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
//     o_msg.ID = 0;
//     o_msg.LEN = 2;
//     o_msg.DATA[0] = 0x81;
//     o_msg.DATA[1] = 0x00 + (uint8_t)n_node_id;

//     Pcan_driver::Write(o_msg);
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//     o_msg.DATA[0] = 0x01;
//     o_msg.DATA[1] = 0x00 + (uint8_t)n_node_id;
//     Pcan_driver::Write(o_msg);
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
// }

// void Pcan_driver_curtis_3d_sd::Write(const Wheel_Cmd_t& st_cmd)
// {
//     int n_target_rpm_f = st_cmd.f_FL_target_rpm;
//     float ff_angle_target = st_cmd.f_FL_target_deg * 100;

//     PCAN_3D_SD_WRITE u_write;

//     u_write.un16_targetRPM = (uint16_t)(abs(n_target_rpm_f));
//     u_write.un8_none = 0;
//     u_write.n16_target_angle = (int16_t)(ff_angle_target);
//     u_write.un8_interlock = 1;
//     u_write.un8_direction = n_target_rpm_f < 0 ? 2 : 1;
//     Pcan_driver::Write(0x223, u_write.byte_space, 8);
// }

// void Pcan_driver_curtis_3d_sd::PcanCallback(const boost::any& any_type_var)
// {
//     TPCANMsg o_canmsg = boost::any_cast<TPCANMsg>(any_type_var);

//     NaviFra::SD_Wheel_Data_t o_data;
//     if (o_canmsg.ID == 0x250) {
//         PCAN_3D_SD_TRACTION_READ u_read;
//         memcpy(u_read.byte_space, o_canmsg.DATA, 8);

//         motor_msgs::MotorData data_fl_traction;
//         data_fl_traction.error_code = (int)u_read.un8_error;
//         data_fl_traction.current = (float)u_read.un16_current;
//         data_fl_traction.voltage = (float)u_read.un16_voltage;
//         data_fl_traction.feedback_velocity = (float)u_read.n16_motor_rpm;
//         n_FL_traction_feedback_rpm_ = o_data.f_FL_traction_feedback_rpm;

//         traction_time_ = std::chrono::system_clock::now();

//         // static std::chrono::system_clock::time_point rpm_time_;
//         // static double d_FL_traction_encoder_rpm_ = 0;

//         // std::chrono::duration<double> rpm_duration = (std::chrono::system_clock::now() - rpm_time_);
//         // d_FL_traction_encoder_rpm_ += o_data.f_FL_traction_feedback_rpm * st_driver_param_.st_sd_param.f_sd_FL_encoder_pulse / 60.0f *
//         //     (st_driver_param_.st_sd_param.f_sd_FL_wheel_diameter_m * M_PI) / st_driver_param_.st_sd_param.f_sd_FL_gear_ratio *
//         //     float(rpm_duration.count());

//         // rpm_time_ = std::chrono::system_clock::now();

//         UpdateMotorData(static_cast<int>(MotorID::FL_TRACTION), data_fl_traction, PDO_ID::TPDO1);  // TPDO1 (상태, 에러, 전류)
//     }

//     if (o_canmsg.ID == 0x251) {
//         PCAN_3D_SD_STEER_READ u_read;
//         memcpy(u_read.byte_space, o_canmsg.DATA, 8);

//         o_data.b_FL_steer_feedback_update = true;
//         o_data.f_FL_steer_angle_deg = (float)(u_read.n16_steer_angle) / 100;
//         o_data.f_FL_steer_error = float(u_read.un8_error);

//         steer_time_ = std::chrono::system_clock::now();

//         Communicator::Notify("motor_callback", o_data);
//     }

//     if (o_canmsg.ID == 0x280 + st_interface_param_.st_pcan_param.n_FL_abssteer_pcan_id) {
//         PCAN_3D_SD_ABSSTEER_READ u_read;
//         memcpy(u_read.byte_space, o_canmsg.DATA, 8);

//         o_data.b_FL_abssteer_feedback_update = true;
//         float f_angle = u_read.n16_encoder_value;

//         if (f_angle > 7200)
//             f_angle -= 14400;
//         f_angle /= -40;
//         // f_angle /= -40;
//         o_data.f_FL_abssteer_angle_deg = f_angle;
//         // cout<<"f_FL_abssteer_angle_deg "<<f_angle<<endl;

//         tp_absencoder_ = std::chrono::system_clock::now();
//         Communicator::Notify("motor_callback", o_data);
//     }
// }

// // 특정 모터 데이터를 가져오는 getter
// motor_msgs::MotorData Pcan_driver_curtis_3d_sd::getMotorData(int motor_id)
// {
//     std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
//     return motor_data_[motor_id];
// }

// // 특정 모터 데이터를 설정하는 setter
// void Pcan_driver_curtis_3d_sd::SetMotorData(int motor_id, const motor_msgs::MotorData& data)
// {
//     std::lock_guard<std::mutex> lock(data_mutex_);  // 멀티스레드 안전
//     motor_data_[motor_id] = data;
// }

// void Pcan_driver_curtis_3d_sd::UpdateMotorData(int motor_id, const motor_msgs::MotorData& update_data, int pdo_type)
// {
//     motor_msgs::MotorData data = getMotorData(motor_id);  // 해당 모터의 데이터를 가져옴

//     data.motor_id = motor_id;
//     auto now = std::chrono::system_clock::now();
//     data.last_update_time = std::chrono::duration<double>(now.time_since_epoch()).count();

//     if (pdo_type == PDO_ID::TPDO1) {  // TPDO1 처리
//         if(motor_id == MotorID::FL_TRACTION) {
//             data.error_code = update_data.error_code;
//             data.current = update_data.current;
//             data.voltage = update_data.voltage;
//             data.feedback_velocity = update_data.feedback_velocity;
//         }
//         else if(motor_id == MotorID::FL_STEER) {
            
//         }
//     }
//     else if (pdo_type == PDO_ID::TPDO2) {  // TPDO2 처리 (속도, 전류)
//     }
//     else if (pdo_type == PDO_ID::TPDO3) {  // TPDO3 처리 (엔코더)
//         data.error_code = update_data.error_code;
//         data.voltage = update_data.voltage / 1000.0;  // mA -> A 변환
//         data.current = update_data.current / 1000.0;  // mA -> A 변환
//         if (data.error_code != 0) {
//             data.is_error = true;
//         }
//         else {
//             data.is_error = false;
//         }
//     }
//     else if (pdo_type == PDO_ID::RPDO1) {  // TPDO3 처리 (엔코더)
//         data.input_velocity = update_data.input_velocity;
//     }
//     SetMotorData(motor_id, data);
// }
