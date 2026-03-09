#include "interface/serial/serial_driver_base.hpp"

#include "core/util/logger.hpp"

using namespace NaviFra;

serial::Serial serial_traction_motor;

void Serial_driver::InterfaceOpen()
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    // Serial 1번 사용
    try {
        serial_traction_motor.setPort(st_interface_param_.st_serial_param.s_serial_tty.c_str());
        // serial_traction_motor.setPort("/dev/ttyUSB_Serial");
        serial_traction_motor.setBaudrate(st_interface_param_.st_serial_param.n_serial_baudrate);
        serial::Timeout serial_1_timeout = serial::Timeout::simpleTimeout(1667);  // 1667 when baud is 57600, 0.6ms
        serial_traction_motor.setTimeout(serial_1_timeout);  // 2857 when baud is 115200, 0.35ms
        serial_traction_motor.open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open Serial port 1");
    }

    // Serial 2번 사용(1개만 사용시 주석 처리 필요)
    // try
    // {
    //     serial_main_io.setPort("/dev/ttyUSB_Serial2");
    //     serial_main_io.setBaudrate(st_interface_param_.st_serial_param.n_serial_2_baudrate);
    //     serial::Timeout serial_2_timeout = serial::Timeout::simpleTimeout(1000); 			//1667 when baud is 57600, 0.6ms
    //     serial_main_io.setTimeout(serial_2_timeout);                                        //2857 when baud is 115200, 0.35ms
    //     serial_main_io.open();
    // }
    // catch (serial::IOException& e)
    // {
    //     ROS_ERROR_STREAM("Unable to open Serial port 2");
    // }

    if (serial_traction_motor.isOpen()) {
        ROS_INFO_STREAM("Serial Port 1 initialized");
    }

    // if(serial_main_io.isOpen()) {
    //     ROS_INFO_STREAM("Serial Port 2 initialized");
    // }

    if (false == th_readLoop_.joinable()) {
        th_readLoop_ = std::thread(&Serial_driver::Read_Loop, this);
    }
}

void Serial_driver::uninit()
{
}

void Serial_driver::reset()
{
}

void Serial_driver::EncoderZero(string& str_data)
{
    LOG_INFO("SetEncoderZero");
}

void Serial_driver::Write(const std::string& str_data)
{
    if (serial_traction_motor.isOpen() == true) {
        serial_traction_motor.write(str_data);
    }
}

void Serial_driver::Write(const uint8_t* un8_data, size_t n_length)
{
    if (serial_traction_motor.isOpen() == true) {
#if 0
		uint8_t data;
		for (int i = 0 ; i < n_length ; i++)
		{
			data = un8_data[i];
			ROS_INFO("command %2d: %02x, %3d", i, data, data);
		}
#endif
        serial_traction_motor.write(un8_data, n_length);
    }
}

void Serial_driver::Read_Loop()
{
    // char stx = 0x02;
    // char etx = 0x03;

    SerialMsg o_serialmsg;
    // std::string str_serial_rec_data = "";
    o_serialmsg.un8_serial_1_rec_buf = new uint8_t[st_interface_param_.st_serial_param.n_serial_rec_buf];

    while (!b_terminate_) {
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        // Serial 1 READ, MD 제공 READ 방법
        o_serialmsg.un8_serial_1_buf_number = serial_traction_motor.available();
        if (o_serialmsg.un8_serial_1_buf_number != 0) {
            if (o_serialmsg.un8_serial_1_buf_number > st_interface_param_.st_serial_param.n_serial_rec_buf) {
                o_serialmsg.un8_serial_1_buf_number = st_interface_param_.st_serial_param.n_serial_rec_buf;
            }
            if (o_serialmsg.un8_serial_1_buf_number == st_interface_param_.st_serial_param.n_serial_rec_buf) {
                serial_traction_motor.read(o_serialmsg.un8_serial_1_rec_buf, o_serialmsg.un8_serial_1_buf_number);
                Notify("SerialCallback", o_serialmsg);
            }
        }
        // else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // }
        std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start_time;
        if (sec.count() * 1000 > 3) {
            LOG_ERROR("SERIALREAD: %f ms", sec.count() * 1000);
        }
    }
    delete[] o_serialmsg.un8_serial_1_rec_buf;
}

bool Serial_driver::Notify(const std::string& str_cbf_name, const boost::any& any_type_var)
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

bool Serial_driver::RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end()) {
        map_callback_pt_[str_cbf_name] = pt_func;
        return true;
    }
    else {
        return false;
    }
}
