#ifndef NC_UNDOCKING_REACHED_CONDITION_H
#define NC_UNDOCKING_REACHED_CONDITION_H
#include <std_msgs/String.h>

namespace NaviFra {
class UnDockingReachedCondition : public BT::ConditionNode {
public:
    UnDockingReachedCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void updateTask();
    ros::Publisher task_alarm_;
    ros::Publisher send_led_;
    ros::Publisher send_startcharge_;
};
}  // namespace NaviFra
#endif  // NC_UNDOCKING_REACHED_CONDITION_H