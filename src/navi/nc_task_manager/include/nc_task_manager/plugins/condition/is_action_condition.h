#ifndef NC_IS_ACTION_CONDITION_H
#define NC_IS_ACTION_CONDITION_H

namespace NaviFra {
class IsActionCondition : public BT::ConditionNode {
public:
    IsActionCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher task_alarm_;
};
}  // namespace NaviFra
#endif