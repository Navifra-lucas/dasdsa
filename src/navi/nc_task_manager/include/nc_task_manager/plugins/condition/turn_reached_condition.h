#ifndef NC_TURN_REACHED_CONDITION_H
#define NC_TURN_REACHED_CONDITION_H
#include <std_msgs/String.h>

namespace NaviFra {
class TurnReachedCondition : public BT::ConditionNode {
public:
    TurnReachedCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void updateTask();
    ros::Publisher task_alarm_;
};
}  // namespace NaviFra
#endif