#include "driver_param.hpp"
namespace NaviFra {
void DriverParam::UpdateParam(
    ros::NodeHandle& nh, ros::NodeHandle& nhp, InterfaceParameters_t& st_interface_param_, DriverParameters_t& st_driver_param_)
{
    // driver param------------------------------------------
    ros::param::param<bool>("driver_base/b_abs_steer_encoder_use", st_driver_param_.b_abs_steer_encoder_use, false);
    ros::param::param<float>("driver_base/f_brake_threshold_rpm", st_driver_param_.f_brake_threshold_rpm, 5);
    ros::param::param<float>("driver_base/f_traction_overcurrent_amp_std", st_driver_param_.f_traction_overcurrent_amp_std, 130);
    ros::param::param<float>("driver_base/f_traction_overcurrent_time_std", st_driver_param_.f_traction_overcurrent_time_std, 3);
    ros::param::param<float>("driver_base/f_steer_overcurrent_amp_std", st_driver_param_.f_steer_overcurrent_amp_std, 130);
    ros::param::param<float>("driver_base/f_steer_overcurrent_time_std", st_driver_param_.f_steer_overcurrent_time_std, 3);

    ros::param::param<float>("driver_wheel/f_FL_wheel_diameter_m", st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m, 0.15);
    ros::param::param<float>("driver_traction/f_FL_traction_encoder_pulse", st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse, 4096.0);
    ros::param::param<float>("driver_traction/f_FL_traction_gear_ratio", st_driver_param_.st_traction_param.f_FL_traction_gear_ratio, 1.0);

    // inter face--------------------------------------------
    
    // PCAN
    ros::param::param<bool>("pcan/b_pcan_use", st_interface_param_.b_pcan_use, false);
    ros::param::param<int>("pcan/n_pcan_mode", st_interface_param_.st_pcan_param.n_pcan_mode, PCAN_MODE::PCAN_CURTIS);
    ros::param::param<int>("pcan/n_pcan_baudrate", st_interface_param_.st_pcan_param.n_pcan_baudrate, PCAN_BAUDRATE::NC_PCAN_BAUD_500K);
    ros::param::param<int>("pcan/n_pcan_bus", st_interface_param_.st_pcan_param.n_pcan_bus, PCAN_BUS::NC_PCAN_USBBUS1);

    // PCAN ID
    ros::param::param<int>("pcan/n_FL_traction_pcan_id", st_interface_param_.st_pcan_param.n_FL_traction_pcan_id, 0x01);
    ros::param::param<int>("pcan/n_FR_traction_pcan_id", st_interface_param_.st_pcan_param.n_FR_traction_pcan_id, 0x02);
    ros::param::param<int>("pcan/n_RL_traction_pcan_id", st_interface_param_.st_pcan_param.n_RL_traction_pcan_id, 0x03);
    ros::param::param<int>("pcan/n_RR_traction_pcan_id", st_interface_param_.st_pcan_param.n_RR_traction_pcan_id, 0x04);
    ros::param::param<int>("pcan/n_FL_steer_pcan_id", st_interface_param_.st_pcan_param.n_FL_steer_pcan_id, 0x05);
    ros::param::param<int>("pcan/n_FR_steer_pcan_id", st_interface_param_.st_pcan_param.n_FR_steer_pcan_id, 0x06);
    ros::param::param<int>("pcan/n_RL_steer_pcan_id", st_interface_param_.st_pcan_param.n_RL_steer_pcan_id, 0x07);
    ros::param::param<int>("pcan/n_RR_steer_pcan_id", st_interface_param_.st_pcan_param.n_RR_steer_pcan_id, 0x08);
    ros::param::param<int>("pcan/n_FL_abssteer_pcan_id", st_interface_param_.st_pcan_param.n_FL_abssteer_pcan_id, 0x09);
    ros::param::param<int>("pcan/n_FR_abssteer_pcan_id", st_interface_param_.st_pcan_param.n_FR_abssteer_pcan_id, 0x0A);
    ros::param::param<int>("pcan/n_RL_abssteer_pcan_id", st_interface_param_.st_pcan_param.n_RL_abssteer_pcan_id, 0x0B);
    ros::param::param<int>("pcan/n_RR_abssteer_pcan_id", st_interface_param_.st_pcan_param.n_RR_abssteer_pcan_id, 0x0C);

    // CAN
    ros::param::param<bool>("can/b_can_use", st_interface_param_.b_can_use, true);
    ros::param::param<int>("can/n_can_mode", st_interface_param_.st_can_param.n_can_mode, CAN_MODE::CAN_AMC);
    // ros::param::param<int>("can/n_project_id", st_interface_param_.st_can_param.n_project_id, LEADSHINE_MODE::DEFAULT);
    ros::param::param<int>("can/n_turn_table_abssteer_can_id", st_interface_param_.st_can_param.n_turn_table_abssteer_can_id, 0x00);

    // CAN ID
    ros::param::param<int>("can/n_FL_traction_can_id", st_interface_param_.st_can_param.n_FL_traction_can_id, 0x01);
    ros::param::param<int>("can/n_FR_traction_can_id", st_interface_param_.st_can_param.n_FR_traction_can_id, 0x03);
    ros::param::param<int>("can/n_RL_traction_can_id", st_interface_param_.st_can_param.n_RL_traction_can_id, 0x04);
    ros::param::param<int>("can/n_RR_traction_can_id", st_interface_param_.st_can_param.n_RR_traction_can_id, 0x02);
    ros::param::param<int>("can/n_FL_steer_can_id", st_interface_param_.st_can_param.n_FL_steer_can_id, 0x05);
    ros::param::param<int>("can/n_FR_steer_can_id", st_interface_param_.st_can_param.n_FR_steer_can_id, 0x06);
    ros::param::param<int>("can/n_RL_steer_can_id", st_interface_param_.st_can_param.n_RL_steer_can_id, 0x07);
    ros::param::param<int>("can/n_RR_steer_can_id", st_interface_param_.st_can_param.n_RR_steer_can_id, 0x08);
    ros::param::param<int>("can/n_FL_abssteer_can_id", st_interface_param_.st_can_param.n_FL_abssteer_can_id, 0x09);
    ros::param::param<int>("can/n_FR_abssteer_can_id", st_interface_param_.st_can_param.n_FR_abssteer_can_id, 0x0A);
    ros::param::param<int>("can/n_RL_abssteer_can_id", st_interface_param_.st_can_param.n_RL_abssteer_can_id, 0x0B);
    ros::param::param<int>("can/n_RR_abssteer_can_id", st_interface_param_.st_can_param.n_RR_abssteer_can_id, 0x0C);
    
    // SERIAL
    ros::param::param<bool>("serial/b_serial_use", st_interface_param_.b_serial_use, false);
    ros::param::param<std::string>("serial/s_serial_tty", st_interface_param_.st_serial_param.s_serial_tty, "/dev/ttyUSB_Serial");
    ros::param::param<int>("serial/n_serial_mode", st_interface_param_.st_serial_param.n_serial_mode, SERIAL_MODE::SERIAL_COMODO);
    ros::param::param<int>(
        "serial/n_serial_baudrate", st_interface_param_.st_serial_param.n_serial_baudrate, SERIAL_BAUDRATE::NC_SERIAL_BAUD_57600);
    ros::param::param<int>("serial/n_serial_rec_buf", st_interface_param_.st_serial_param.n_serial_rec_buf, 40);

    if(!b_robot_init_)
    {   
        LOG_INFO("//-------------------PRINT ALL DRIVER PARAM-----------------------//\n");
        LOG_INFO("driver_base/b_abs_steer_encoder_use %d\n", st_driver_param_.b_abs_steer_encoder_use);
        LOG_INFO("driver_base/f_brake_threshold_rpm %f\n", st_driver_param_.f_brake_threshold_rpm);
        LOG_INFO("driver_base/f_traction_overcurrent_amp_std %f\n", st_driver_param_.f_traction_overcurrent_amp_std);
        LOG_INFO("driver_base/f_traction_overcurrent_time_std %f\n", st_driver_param_.f_traction_overcurrent_time_std);
        LOG_INFO("driver_base/f_steer_overcurrent_amp_std %f\n", st_driver_param_.f_steer_overcurrent_amp_std);
        LOG_INFO("driver_base/f_steer_overcurrent_time_std %f\n", st_driver_param_.f_steer_overcurrent_time_std);

        LOG_INFO("//-----------------------------PCAN-------------------------------//\n");
        LOG_INFO("pcan/b_pcan_use %d\n", st_interface_param_.b_pcan_use);
        LOG_INFO("pcan/n_pcan_mode %d\n", st_interface_param_.st_pcan_param.n_pcan_mode);
        LOG_INFO("pcan/n_pcan_baudrate %d\n", st_interface_param_.st_pcan_param.n_pcan_baudrate);
        LOG_INFO("pcan/n_pcan_bus %d\n", st_interface_param_.st_pcan_param.n_pcan_bus);

        LOG_INFO("//-----------------------------CAN--------------------------------//\n");
        LOG_INFO("can/b_can_use %d\n", st_interface_param_.b_can_use);
        LOG_INFO("can/n_can_mode %d\n", st_interface_param_.st_can_param.n_can_mode);
        // LOG_INFO("can/n_project_id %d\n", st_interface_param_.st_can_param.n_project_id);
        LOG_INFO("can/n_turn_table_abssteer_can_id %d\n", st_interface_param_.st_can_param.n_turn_table_abssteer_can_id);

        LOG_INFO("//----------------------------SERIAL------------------------------//\n");
        LOG_INFO("serial/b_serial_use %d\n", st_interface_param_.b_serial_use);
        LOG_INFO("serial/s_serial_tty %s\n", st_interface_param_.st_serial_param.s_serial_tty.c_str());
        LOG_INFO("serial/n_serial_mode %d\n", st_interface_param_.st_serial_param.n_serial_mode);
        LOG_INFO("serial/n_serial_baudrate %d\n", st_interface_param_.st_serial_param.n_serial_baudrate);
        LOG_INFO("serial/n_serial_rec_buf %d\n", st_interface_param_.st_serial_param.n_serial_rec_buf);

        LOG_INFO("//---------------------------PCAN ID------------------------------//\n");
        LOG_INFO("pcan/n_FL_traction_pcan_id %d\n", st_interface_param_.st_pcan_param.n_FL_traction_pcan_id);
        LOG_INFO("pcan/n_FR_traction_pcan_id %d\n", st_interface_param_.st_pcan_param.n_FR_traction_pcan_id);
        LOG_INFO("pcan/n_RL_traction_pcan_id %d\n", st_interface_param_.st_pcan_param.n_RL_traction_pcan_id);
        LOG_INFO("pcan/n_RR_traction_pcan_id %d\n", st_interface_param_.st_pcan_param.n_RR_traction_pcan_id);
        LOG_INFO("pcan/n_FL_steer_pcan_id %d\n", st_interface_param_.st_pcan_param.n_FL_steer_pcan_id);
        LOG_INFO("pcan/n_FR_steer_pcan_id %d\n", st_interface_param_.st_pcan_param.n_FR_steer_pcan_id);
        LOG_INFO("pcan/n_RL_steer_pcan_id %d\n", st_interface_param_.st_pcan_param.n_RL_steer_pcan_id);
        LOG_INFO("pcan/n_RR_steer_pcan_id %d\n", st_interface_param_.st_pcan_param.n_RR_steer_pcan_id);
        LOG_INFO("pcan/n_FL_abssteer_pcan_id %d\n", st_interface_param_.st_pcan_param.n_FL_abssteer_pcan_id);
        LOG_INFO("pcan/n_FR_abssteer_pcan_id %d\n", st_interface_param_.st_pcan_param.n_FR_abssteer_pcan_id);
        LOG_INFO("pcan/n_RL_abssteer_pcan_id %d\n", st_interface_param_.st_pcan_param.n_RL_abssteer_pcan_id);
        LOG_INFO("pcan/n_RR_abssteer_pcan_id %d\n", st_interface_param_.st_pcan_param.n_RR_abssteer_pcan_id);

        LOG_INFO("//--------------------------CAN ID--------------------------------//\n");
        LOG_INFO("can/n_FL_traction_can_id %d\n", st_interface_param_.st_can_param.n_FL_traction_can_id);
        LOG_INFO("can/n_FR_traction_can_id %d\n", st_interface_param_.st_can_param.n_FR_traction_can_id);
        LOG_INFO("can/n_RL_traction_can_id %d\n", st_interface_param_.st_can_param.n_RL_traction_can_id);
        LOG_INFO("can/n_RR_traction_can_id %d\n", st_interface_param_.st_can_param.n_RR_traction_can_id);
        LOG_INFO("can/n_FL_steer_can_id %d\n", st_interface_param_.st_can_param.n_FL_steer_can_id);
        LOG_INFO("can/n_FR_steer_can_id %d\n", st_interface_param_.st_can_param.n_FR_steer_can_id);
        LOG_INFO("can/n_RL_steer_can_id %d\n", st_interface_param_.st_can_param.n_RL_steer_can_id);
        LOG_INFO("can/n_RR_steer_can_id %d\n", st_interface_param_.st_can_param.n_RR_steer_can_id);
        LOG_INFO("can/n_FL_abssteer_can_id %d\n", st_interface_param_.st_can_param.n_FL_abssteer_can_id);
        LOG_INFO("can/n_FR_abssteer_can_id %d\n", st_interface_param_.st_can_param.n_FR_abssteer_can_id);
        LOG_INFO("can/n_RL_abssteer_can_id %d\n", st_interface_param_.st_can_param.n_RL_abssteer_can_id);
        LOG_INFO("can/n_RR_abssteer_can_id %d\n", st_interface_param_.st_can_param.n_RR_abssteer_can_id);

    }
    b_robot_init_ = true;
}

};  // namespace NaviFra
