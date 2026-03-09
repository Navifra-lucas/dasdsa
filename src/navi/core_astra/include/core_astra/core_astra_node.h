#ifndef CORE_ASTRA_CORE_ASTRA_NODE_H
#define CORE_ASTRA_CORE_ASTRA_NODE_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/timer.h>

namespace NaviFra {

class ZMQHandler;
class CoreAstraNode {
public:
    CoreAstraNode();
    ~CoreAstraNode();

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Publisher ros_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;
    ros::Timer status_timer_;  // <-- 새로운 멤버 변수 추가 필요

    // MemoryStateStorage storage_;
    // std::unique_ptr<ZMQHandler> handler_;
};
}  // namespace NaviFra

#endif  // CORE_ASTRA_CORE_ASTRA_NODE_H