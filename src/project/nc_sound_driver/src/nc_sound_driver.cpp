#include "nc_sound_driver.hpp"
using namespace NaviFra;

SoundDriver::SoundDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    NLOG(info) << "SoundDriver Create";
    int sound_param = 0;
    nhp_.param<int>("n_sound_volume", sound_param, 10);

    n_sound_volume_ = VolumeConvert(sound_param);

    o_can_driver_.InterfaceOpen();
    RegistListener();

    // th_heartbeat_ = std::thread(&SoundDriver::HeartBeatLoop, this);
}

SoundDriver::~SoundDriver()
{
    b_terminate_ = true;
    SoundStop();
    NLOG(info) << "SoundDriver Destroyed";
}

void SoundDriver::RegistListener()
{
    sub_sound_ = nh_.subscribe("/navifra/sound", 5, &SoundDriver::SubscribeSoundCmd, this);
    sub_sound_volume_ = nh_.subscribe("/navifra/sound_volume", 5, &SoundDriver::SubscribeSoundVolumeCmd, this);
}

// void SoundDriver::HeartBeatLoop()
// {
//     while (!b_terminate_) {
//         if(!b_start_sound_) {
//             std::chrono::duration<double> sec = std::chrono::steady_clock::now() - checktime_start;
//             if(sec.count() > 2) {
//                 b_start_sound_ = true;
//             }
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(20));
//     }
// }

int SoundDriver::VolumeConvert(int n_volume)
{
    if (n_volume < 0)
        n_volume = 0;
    else if (n_volume > 100)
        n_volume = 100;

    return (n_volume * 28) / 100;
}

void SoundDriver::SubscribeSoundVolumeCmd(const std_msgs::Byte::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_sound_);
    auto value = msg->data;
    n_sound_volume_ = VolumeConvert((uint8_t)value);
}

void SoundDriver::SendSoundCommand(uint8_t sound_status)
{
    if (sound_status == SOUND::SOUND_STOP) {
        SoundStop();
        return;
    }
    uint8_t n_sound_volume;
    {
        std::lock_guard<std::mutex> lock(mtx_sound_);
        n_sound_volume = n_sound_volume_;
    }

    can_frame frame;
    frame.can_id = 0x201;
    frame.can_dlc = 8;
    memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = 0x01;  // 시작 byte
    frame.data[1] = 0x51;  // function (0x51 = 동작)
    frame.data[2] = sound_status;  // 상태 바이트 위치
    frame.data[3] = n_sound_volume;  // volume
    frame.data[4] = 0x00;  // 횟수(0 = 반복, 1 = 1회)
    // if(sound_status == SOUND::SOUND_DOCKING_IN_START || sound_status == SOUND::SOUND_DOCKING_OUT_START || sound_status ==
    // SOUND::SOUND_CHARGING_START ||
    //     sound_status == SOUND::SOUND_DOCKING_IN_START_EN || sound_status == SOUND::SOUND_DOCKING_OUT_START_EN || sound_status ==
    //     SOUND::SOUND_CHARGING_START_EN) { frame.data[4] = 0x01;  // 횟수(0 = 반복, 1 = 1회)
    // }
    // else {
    //     checktime_start = std::chrono::steady_clock::now();
    // }
    frame.data[5] = 0x00;  // reserve

    // checksum 계산 (0~5번 byte XOR)
    uint8_t checksum = 0;
    for (int i = 0; i <= 5; i++) {
        checksum ^= frame.data[i];
    }
    frame.data[6] = checksum;

    frame.data[7] = 0x02;  // 종료 byte

    o_can_driver_.Write(frame);
}

void SoundDriver::SoundStop()
{
    can_frame frame;
    frame.can_id = 0x201;
    frame.can_dlc = 8;
    memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = 0x01;  // 시작 byte
    frame.data[1] = 0x51;  // function (0x51 = 동작)
    frame.data[2] = 0x00;  // 상태 바이트 위치
    frame.data[3] = 0x00;  // volume
    frame.data[4] = 0x00;  // 횟수(0 = 반복, 1 = 1회)
    frame.data[5] = 0x00;  // reserve

    // checksum 계산 (0~5번 byte XOR)
    uint8_t checksum = 0;
    for (int i = 0; i <= 5; i++) {
        checksum ^= frame.data[i];
    }
    frame.data[6] = checksum;

    frame.data[7] = 0x02;  // 종료 byte

    o_can_driver_.Write(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void SoundDriver::SubscribeSoundCmd(const std_msgs::String::ConstPtr& msg)
{
    static std::string cmd_pre = "";
    std::string cmd = msg->data;
    // if(!b_start_sound_) {
    //     if(cmd == "docking") {
    //         cmd = "docking_start";
    //     }
    //     else if(cmd == "docking_out") {
    //         cmd = "docking_out_start";
    //     }
    //     else if(cmd == "charging") {
    //         cmd = "charging_start";
    //     }
    // }
    // else if(cmd_pre == "docking" || cmd_pre == "docking_out" || cmd_pre == "charging") {
    //     if(cmd_pre != cmd) {
    //         b_start_sound_ = false;
    //     }
    // }
    if (cmd_pre != cmd) {
        NLOG(info) << "SOUND Command Received: " << cmd;
    }
    std::map<std::string, uint8_t> sound_map = {
        {"sound_stop", SOUND::SOUND_STOP},  // 사운드 정지
        {"unlisted", SOUND::SOUND_UNLISTED},  // 정의되지않음
        {"init", SOUND::SOUND_INIT},  // init
        {"moving", SOUND::SOUND_DRIVING},
        {"docking_start", SOUND::SOUND_DOCKING_IN_START},
        {"docking", SOUND::SOUND_DOCKING},
        {"docking_out_start", SOUND::SOUND_DOCKING_OUT_START},
        {"docking_out", SOUND::SOUND_DOCKING_OUT},
        {"workingonboard", SOUND::SOUND_WORKING},
        {"standby", SOUND::SOUND_STANDBY},
        {"error", SOUND::SOUND_EMG},
        {"charging_start", SOUND::SOUND_CHARGING_START},
        {"charging", SOUND::SOUND_CHARGING},
        {"pause", SOUND::SOUND_PAUSE},
        {"docking_in_start_en", SOUND::SOUND_DOCKING_IN_START_EN},
        {"docking_out_start_en", SOUND::SOUND_DOCKING_OUT_START_EN},
        {"charging_start_en", SOUND::SOUND_CHARGING_START_EN},
        {"idle", SOUND::SOUND_STOP},  // 사운드 정지
    };

    auto it = sound_map.find(cmd);
    if (it != sound_map.end()) {
        SendSoundCommand(it->second);
    }
    else {
        NLOG(warning) << "Unknown SOUND command: " << cmd;
    }
    cmd_pre = cmd;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_sound_driver");
    ros::NodeHandle nh, nhp("~");
    NaviFra::SoundDriver node(nh, nhp);
    ros::spin();
    return 0;
}
