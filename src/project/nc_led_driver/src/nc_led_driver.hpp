#ifndef NAVIFRA_LED_DRIVER_HPP_
#define NAVIFRA_LED_DRIVER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include "interface/can/can_driver_base.hpp"
#include "core/util/logger.hpp"
#include <core_msgs/BmsInfo.h>
#include <core_msgs/HacsNodeList.h>


namespace NaviFra {

class LedDriver {
public:
    LedDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~LedDriver();

private:
    // ROS 통신
    void RegistListener();
    void SubscribeLedCmd(const std_msgs::String::ConstPtr& msg);

    // CAN 전송
    void SendLedCommand(uint8_t led_status);
    void bmsCallback(const core_msgs::BmsInfo::ConstPtr& msg);
    void dockingledCallback(const std_msgs::Int64::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber sub_led_;
    ros::Subscriber sub_bms_info_;
    ros::Subscriber sub_led_docking_;

    Can_driver o_can_driver_;  // CAN 드라이버 객체

    int n_can_bus_led_ = 0;
    int n_can_baud_led_ = 500;
    uint8_t n_front_led_ = 0;
    uint8_t n_rear_led_ = 0;
    float f_soc_ = 0.0f;  // BMS에서 받은 SOC 값
    bool b_terminate_ = false;
    string docking_type_ = "";
    int n_docking_led_ = 0;
};

} // namespace NaviFra

#endif // NAVIFRA_LED_DRIVER_HPP_
