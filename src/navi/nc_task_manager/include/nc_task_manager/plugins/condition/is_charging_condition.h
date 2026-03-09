#ifndef NC_IS_CHAGING_CONDITION_H
#define NC_IS_CHAGING_CONDITION_H

namespace NaviFra {
class IsChargingCondition : public BT::ConditionNode {
public:
    IsChargingCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher task_alarm_;
    ros::Publisher send_charging_;
};
}  // namespace NaviFra
#endif //NC_IS_CHAGING_CONDITION_H
