#ifndef NC_IS_CHECKING_CONDITION_H
#define NC_IS_CHECKING_CONDITION_H

namespace NaviFra {
class IsCheckingCondition : public BT::ConditionNode {
public:
    IsCheckingCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher task_alarm_;
    ros::Publisher send_checking_;
    ros::Publisher send_speaker_;
};
}  // namespace NaviFra
#endif //NC_IS_CHECKING_CONDITION_H
