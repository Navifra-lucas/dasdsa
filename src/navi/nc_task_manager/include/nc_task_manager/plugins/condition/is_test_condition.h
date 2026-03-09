#ifndef NC_IS_TEST_CONDITION_H
#define NC_IS_TEST_CONDITION_H

namespace NaviFra {
class IsTestCondition : public BT::ConditionNode {
public:
    IsTestCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher task_alarm_;
    ros::Publisher send_turn_;
};
}  // namespace NaviFra
#endif