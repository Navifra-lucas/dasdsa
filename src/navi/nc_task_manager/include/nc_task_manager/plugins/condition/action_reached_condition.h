#ifndef NC_ACTION_REACHED_CONDITION_H
#define NC_ACTION_REACHED_CONDITION_H
#include <std_msgs/String.h>
#include <chrono>

namespace NaviFra {
class ActionReachedCondition : public BT::ConditionNode {
public:
    ActionReachedCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void updateTask();
    ros::Publisher task_alarm_;
    std::chrono::system_clock::time_point tp_action_ = std::chrono::system_clock::now();
};
}  // namespace NaviFra
#endif