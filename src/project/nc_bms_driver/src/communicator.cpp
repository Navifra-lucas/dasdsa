#include "interface/communicator.hpp"

namespace NaviFra {

Communicator::~Communicator()
{
    if (!b_terminate_) {  // Terminate()가 호출되지 않은 경우에만
        b_terminate_ = true;
        LOG_INFO("~wait thread Communicator");
        if (th_readLoop_.joinable()) {
            th_readLoop_.join();
        }
    }
    LOG_INFO("~Communicator");
}
void Communicator::Terminate()
{
    b_terminate_ = true;
    if (th_readLoop_.joinable()) {
        LOG_INFO("Waiting for communicator thread to terminate");
        th_readLoop_.join();
    }
}
void Communicator::SetInterfaceParam(const InterfaceParameters_t& st_interface_param)
{
    st_interface_param_ = st_interface_param;
}

void Communicator::SetDriverParam(const DriverParameters_t& st_driver_param)
{
    st_driver_param_ = st_driver_param;
}

bool Communicator::Notify(const std::string& str_cbf_name, const boost::any& any_type_var)
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

bool Communicator::RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end()) {
        map_callback_pt_[str_cbf_name] = pt_func;
        return true;
    }
    else {
        return false;
    }
}
}  // namespace NaviFra