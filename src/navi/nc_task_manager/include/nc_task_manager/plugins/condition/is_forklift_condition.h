#ifndef NC_IS_FORKLIFT_CONDITION_H
#define NC_IS_FORKLIFT_CONDITION_H

namespace NaviFra {
class IsForkLiftCondition : public BT::ConditionNode {
public:
    IsForkLiftCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher task_alarm_;
    ros::Publisher fork_lift_;
};
}  // namespace NaviFra
#endif  // NC_IS_FORKLIFT_CONDITION_H