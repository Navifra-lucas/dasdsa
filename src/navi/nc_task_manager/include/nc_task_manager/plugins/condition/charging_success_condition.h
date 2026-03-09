#ifndef NC_CHARGING_SUCCESS_CONDITION_H
#define NC_CHARGING_SUCCESS_CONDITION_H

namespace NaviFra {
class ChargingSuccessCondition : public BT::ConditionNode {
public:
    ChargingSuccessCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void updateTask();
    ros::Publisher task_alarm_;
    ros::Publisher send_charging_;
};
}  // namespace NaviFra
#endif// NC_CHARGING_SUCCESS_CONDITION_H
