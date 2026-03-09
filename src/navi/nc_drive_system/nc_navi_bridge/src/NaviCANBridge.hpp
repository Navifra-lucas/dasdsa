#ifndef NAVIFRA_NAVICAN_BRIDGE_H
#define NAVIFRA_NAVICAN_BRIDGE_H

#include "std_msgs/String.h"
#include "util/logger.hpp"

#include <NaviCAN/NaviCANCore.h>
#include <boost/serialization/singleton.hpp>
#include <motor_msgs/MotorCmd.h>
#include <motor_msgs/MotorCmdRequest.h>
#include <motor_msgs/MotorCmdResponse.h>
#include <motor_msgs/MotorData.h>
#include <motor_msgs/MotorDataInfo.h>
#include <motor_msgs/MotorTarget.h>
#include <motor_msgs/MotorTargetInfo.h>
#include <ros/ros.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

namespace NaviFra {

namespace NaviCAN {
namespace Bridge {
// 모터가 Enable 상태가 되기까지 기다리는 시간 (초)
static constexpr int TIMEOUT_WAITING_FOR_NODES_ENABLED = 20;
}  // namespace Bridge
}  // namespace NaviCAN

class NaviCANBridge : public boost::serialization::singleton<NaviCANBridge> {
    
    friend class boost::serialization::singleton<NaviCANBridge>;

public:
    explicit NaviCANBridge(ros::NodeHandle& nh, ros::NodeHandle& nhp)
        : nh_(nh)
        , nhp_(nhp)
    {
    }

    bool initialize();
    void finalize();

private:
    void ReceivedMotorInfo();
    void RecvEncoderZero(const std_msgs::String::ConstPtr& msg);
    void SubscribeCmdvel(const motor_msgs::MotorTargetInfo::ConstPtr& msg);
    bool SubscribeMotorCmd(motor_msgs::MotorCmdRequest& req, motor_msgs::MotorCmdResponse& res);

    bool WaitForNodesEnabled(void);

    ros::NodeHandle& nh_;
    ros::NodeHandle& nhp_;
    std::shared_ptr<NaviCANCore> navi_can_;

    // motor info data
    motor_msgs::MotorDataInfo motor_data_info_;
    std::array<int32_t, 30> prev_encoder_{};
    std::stringstream status_stream_;

    // 중복 실행 방지용 뮤텍스
    std::mutex motor_info_mutex_;
    std::mutex motor_cmd_mutex_;
    std::mutex cmdvel_mutex_;

    // 스레드 관련
    std::atomic<bool> running_{false};
    std::thread motor_info_thread_;

    // ROS publishers, subscribers, and services
    ros::Publisher pub_motor_info_;
    ros::Subscriber sub_motor_target_;
    ros::Subscriber sub_enc_zero_;
    ros::ServiceServer srv_motor_cmd_;
};

}  // namespace NaviFra

#endif  // NAVIFRA_NAVICAN_BRIDGE_H