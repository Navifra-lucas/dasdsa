#ifndef NC_IS_WAIT_MOVE_CONDITION_H
#define NC_IS_WAIT_MOVE_CONDITION_H

namespace NaviFra {
class IsWaitCondition : public BT::ConditionNode {
public:
    IsWaitCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher task_alarm_;
};
}  // namespace NaviFra
#endif  // NC_IS_WAIT_MOVE_CONDITION_H