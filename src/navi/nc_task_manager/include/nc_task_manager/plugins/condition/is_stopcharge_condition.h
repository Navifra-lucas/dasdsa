#ifndef NC_IS_STOPCHARGE_CONDITION_H
#define NC_IS_STOPCHARGE_CONDITION_H

#include <core_msgs/TaskAlarm.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <chrono>

namespace NaviFra {

class IsStopChargeCondition : public BT::ConditionNode {
public:
    IsStopChargeCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher send_stopcharge_;
    ros::Publisher task_alarm_;
};
}  // namespace NaviFra
#endif