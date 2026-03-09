#ifndef NC_IS_MANUAL_CONDITION_H
#define NC_IS_MANUAL_CONDITION_H

namespace NaviFra {
class IsManualCondition : public BT::ConditionNode {
public:
    IsManualCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher publiser_;
    Task current_task_;
};
}  // namespace NaviFra
#endif