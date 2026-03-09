#ifndef NAVIFRA_SOUND_DRIVER_HPP_
#define NAVIFRA_SOUND_DRIVER_HPP_

#include "core/util/logger.hpp"
#include "interface/can/can_driver_base.hpp"

#include <core_msgs/BmsInfo.h>
#include <core_msgs/HacsNodeList.h>
#include <ros/ros.h>
#include <std_msgs/Byte.h>
#include <std_msgs/String.h>

#include <chrono>

namespace NaviFra {

enum SOUND
{
    SOUND_STOP = 0,
    SOUND_UNLISTED,
    SOUND_INIT,
    SOUND_DRIVING,
    SOUND_DOCKING_IN_START,
    SOUND_DOCKING,
    SOUND_DOCKING_OUT_START,
    SOUND_DOCKING_OUT,
    SOUND_WORKING,
    SOUND_STANDBY,
    SOUND_EMG,
    SOUND_CHARGING_START,
    SOUND_CHARGING,
    SOUND_PAUSE,
    SOUND_DOCKING_IN_START_EN,
    SOUND_DOCKING_OUT_START_EN,
    SOUND_CHARGING_START_EN
};

class SoundDriver {
public:
    SoundDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~SoundDriver();

private:
    // ROS 통신
    void RegistListener();
    void SubscribeSoundVolumeCmd(const std_msgs::Byte::ConstPtr& data);
    void SubscribeSoundCmd(const std_msgs::String::ConstPtr& msg);
    int VolumeConvert(int n_volume);

    // CAN 전송
    void SendSoundCommand(uint8_t sound_status);
    void SoundStop();

    void HeartBeatLoop();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber sub_sound_;
    ros::Subscriber sub_sound_volume_;

    Can_driver o_can_driver_;  // CAN 드라이버 객체

    int n_can_bus_led_ = 0;
    int n_can_baud_led_ = 500;
    int n_sound_volume_ = 0;
    bool b_terminate_ = false;
    bool b_sound_play_state_ = false;
    bool b_start_sound_ = false;
    int n_sound_state_ = 0;

    std::mutex mtx_sound_;

    // std::thread th_heartbeat_;
    // std::chrono::steady_clock::time_point checktime_start = std::chrono::steady_clock::now();
};

}  // namespace NaviFra

#endif  // NAVIFRA_LED_DRIVER_HPP_
