#ifndef NAVIFRA_NAVICAN_NODE_HPP
#define NAVIFRA_NAVICAN_NODE_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <NaviCAN/NaviCANCore.h>
#include <motor_msgs/MotorCmd.h>
#include <motor_msgs/NavicanData.h>
#include <motor_msgs/NavicanDataArray.h>
#include <motor_msgs/MotorTarget.h>
#include <motor_msgs/MotorTargetInfo.h>
#include <motor_msgs/MotorCmdRequest.h>
#include <motor_msgs/MotorCmdResponse.h>

#include <atomic>
#include <sstream>
#include <thread>
#include <chrono>
#include <array>
#include "util/logger.hpp"

namespace NaviFra {
namespace NaviCAN {
namespace Node {

class NaviCANNode {

    public:
    explicit NaviCANNode(ros::NodeHandle& nh,ros::NodeHandle& nhp): nh_(nh), nhp_(nhp) {}

    bool initialize();
    bool finalize();

private:

    bool WaitForMotorsEnabled();

    void PublishMsgs(void);
    void SubscribeTarget(const motor_msgs::MotorTargetInfo::ConstPtr& msg);
    void SubscribeEncoderZero(const std_msgs::String::ConstPtr& msg);
    bool ServiceCmd(motor_msgs::MotorCmdRequest& req, motor_msgs::MotorCmdResponse& res);

    ros::NodeHandle& nh_;
    ros::NodeHandle& nhp_;

    std::unique_ptr<NaviCANCore> navican_;

    // 중복 실행 방지용
    struct Guard {
        std::atomic<bool>& flag;
        Guard(std::atomic<bool>& f) : flag(f) {}
        ~Guard() { flag.store(false); }
    };

    std::atomic<bool> is_updating_msgs_ {false};
    std::atomic<bool> is_processing_cmd_ {false};
    std::atomic<bool> is_updating_target_ {false};
    std::atomic<bool> is_updating_encoder_ {false};

    // motor_msg
    motor_msgs::NavicanDataArray msgs_;
    std::stringstream status_word_stream_;
    std::array<int32_t, 30> prev_external_encoders_ {};

    // 스레드
    std::atomic<bool> msgs_running_ {false};
    std::thread msgs_thread_;

    // ROS publishers, subscribers, and services
    ros::Publisher pub_msgs_;
    ros::Subscriber sub_target_;
    ros::Subscriber sub_encoder_zero_;
    ros::ServiceServer srv_cmd_;

    
    static constexpr uint32_t ROS_QUEUE_SIZE = 5;
    
    // 모터가 Enable 상태가 되기까지 기다리는 시간 (초)
    static constexpr int TIMEOUT_WAITING_FOR_NODES_ENABLED = 10;

};

}
}
}

#endif