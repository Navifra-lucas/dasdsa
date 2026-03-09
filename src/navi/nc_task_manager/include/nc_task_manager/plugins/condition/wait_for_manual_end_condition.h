#ifndef NC_WAIT_FOR_MANUAL_CONDITION_H
#define NC_WAIT_FOR_MANUAL_CONDITION_H
#include <std_msgs/String.h>

namespace NaviFra {
class WaitForManualCondition : public BT::ConditionNode {
public:
    WaitForManualCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void updateTask();
    ros::Publisher task_alarm_;
};
}  // namespace NaviFra
#endif