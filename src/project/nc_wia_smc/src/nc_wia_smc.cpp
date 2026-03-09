#include "nc_wia_smc.hpp"
using namespace NaviFra;

WiaSMC::WiaSMC(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
    , current_state_(RobotState::STATE_ON)
{
    RegistTalker();
    RegistListener();
    InitializeState();
    th_active_ = true;
    state_thread_ = std::thread(&WiaSMC::updateStateLoop, this);

    NLOG(info)<<"WiaSMC Create";
}

WiaSMC::~WiaSMC()
{
    th_active_ = false;
    NLOG(info)<<"destructor";
}

void WiaSMC::InitializeState() {
    NLOG(info) << "Initialize State: " << toString(current_state_);
    setLED(current_state_);
    setSound(current_state_);
}

void WiaSMC::RegistTalker() {
    pub_led_ = nh_.advertise<std_msgs::String>("/navifra/led", 10);
    pub_sound_ = nh_.advertise<std_msgs::String>("/navifra/sound", 10);
    pub_charge_ = nh_.advertise<std_msgs::Bool>("/navifra/charge_on", 10);
}

void WiaSMC::RegistListener() {
    sub_naviinfo_ = nh_.subscribe("/navifra/info", 10, &WiaSMC::stateCallback, this);
    sub_now_task_ = nh_.subscribe("/wia_agent/now_task", 10, &WiaSMC::wiataskCallback, this);
    sub_bmsinfo_ = nh_.subscribe("/bms_info", 10, &WiaSMC::bmsCallback, this);
    sub_output_cmd_ = nh_.subscribe("output_command", 10, &WiaSMC::outputCommandCallback, this);
    sub_plc_info_ = nh_.subscribe("plc_info", 10, &WiaSMC::plcInfoCallback, this);

    /*
        brake, action, mode
    */
}

void WiaSMC::updateStateLoop() {
    NLOG(info)<<"update state loop";
    RobotState robot_state = RobotState::STATE_IDLE;
    RobotState wia_state = RobotState::STATE_IDLE;
    RobotState bms_state = RobotState::STATE_IDLE;
    while (th_active_) {
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            robot_state = robot_state_;
            wia_state = wia_state_;
            bms_state = bms_state_;
        }
        bool b_charge = false;
        if (isCritical(robot_state) || isPuased(robot_state)) {
            current_state_ = robot_state;
        }
        else if (isCharging(bms_state)) {
            current_state_ = bms_state;
        }
        else if (isTasking(wia_state)) {
            current_state_ = wia_state;
        }
        else {
            current_state_ = robot_state;
        }

        if(current_state_ == RobotState::STATE_CHARGING)
        {
            b_charge = true;
        }
        std_msgs::Bool msg;
        msg.data = b_charge;
        pub_charge_.publish(msg);
        setLED(current_state_);
        setSound(current_state_);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

bool WiaSMC::isCritical(RobotState s)
{
    return s == RobotState::STATE_EMERGENCY || s == RobotState::STATE_ERROR;
}

bool WiaSMC::isPuased(RobotState s)
{
    return s == RobotState::STATE_OBSTACLE || s == RobotState::STATE_PAUSE;
}

bool WiaSMC::isTasking(RobotState s)
{
    return s == RobotState::STATE_DOCKING || s == RobotState::STATE_STANDBY || s == RobotState::STATE_PIN || s == RobotState::STATE_DOCKING_OUT || s == RobotState::STATE_CHARGING;
}

bool WiaSMC::isCharging(RobotState s)
{
    return s == RobotState::STATE_CHARGING;
}

void WiaSMC::stateCallback(const core_msgs::NavicoreStatus::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    std::string s_now_task = msg->s_status;
    RobotState new_state = convertState(msg->s_status);
    static std::string s_pre_task = "";
    if(s_now_task != s_pre_task) {
        NLOG(info) << "Robot state msg, new : " << s_now_task << " , " << toString(new_state);
        robot_state_ = new_state;
    }
    s_pre_task = s_now_task;
}

void WiaSMC::wiataskCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    std::string s_now_task = msg->data;
    RobotState new_state = convertState(msg->data);
    static std::string s_pre_task = "";
    if(s_now_task != s_pre_task) {
        NLOG(info) <<"Wia Now Task : "<<s_now_task<<","<<toString(new_state);
        wia_state_ = new_state;
    }
    s_pre_task = s_now_task;
}

void WiaSMC::plcInfoCallback(const core_msgs::PLCInfo::ConstPtr& msg) {
    if(msg->manual_charge_on) {
        b_manual_charge_ = true;
    }
    else {
        b_manual_charge_ = false;
    }
}

void WiaSMC::bmsCallback(const core_msgs::BmsInfo::ConstPtr& msg) {
    // NLOG(info) << "BMS state: " << static_cast<int>(msg->un8_state);
    if (msg->f32_pack_current > 0 && b_manual_charge_) {
        b_charge_start_ = true;
        bms_state_ = RobotState::STATE_CHARGING;
    }
    else {
        b_charge_start_ = false;
        bms_state_ = RobotState::STATE_IDLE;
    }
}

void WiaSMC::outputCommandCallback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "charge_start") {
        b_charge_start_ = true;
    }
    else if(msg->data == "charge_stop") {
        b_charge_start_ = false;
    }
}

void WiaSMC::setLED(RobotState state) {
    std_msgs::String msg;
    msg.data = getStateColor(state);
    NLOG(info) << "Publish LED: " << msg.data;
    pub_led_.publish(msg);
}

void WiaSMC::setSound(RobotState state) {
    std_msgs::String msg;
    msg.data = getStateSound(state);
    if(msg.data=="") return;

    NLOG(info) << "sound state : " << msg.data <<"," << toString(state);
    
    if (msg.data.empty()){
        NLOG(info) << "No sound for state: " << toString(state);
        return;
    }

    NLOG(info) << "Publish Sound: " << msg.data;
    pub_sound_.publish(msg);
    // std::string new_sound = getStateSound(state);
    // std::string prev_sound = getStateSound(current_state_);

    // if (!prev_sound.empty() && prev_sound != new_sound) {
    //     std_msgs::String stop_msg;
    //     stop_msg.data = prev_sound + "_stop";
    //     NLOG(info) << "Publish Stop Sound: " << stop_msg.data;
    //     pub_sound_.publish(stop_msg);
    // }

    // if (!new_sound.empty()) {
    //     msg.data = new_sound + "_start";
    //     NLOG(info) << "Publish Start Sound: " << msg.data;
    //     pub_sound_.publish(msg);
    // } else {
    //     NLOG(info) << "No sound for state: " << toString(state);
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_wia_smc");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    WiaSMC nc_wia_smc(nh, nhp);
    ros::spin();

    return 0;
}