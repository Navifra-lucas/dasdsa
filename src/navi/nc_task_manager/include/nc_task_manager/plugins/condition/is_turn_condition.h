#ifndef NC_IS_TURN_CONDITION_H
#define NC_IS_TURN_CONDITION_H

namespace NaviFra {
class IsTurnCondition : public BT::ConditionNode {
public:
    IsTurnCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher task_alarm_;
    ros::Publisher send_turn_;
};
}  // namespace NaviFra
#endif