#ifndef NC_CHECKING_SUCCESS_CONDITION_H
#define NC_CHECKING_SUCCESS_CONDITION_H

namespace NaviFra {
class CheckingSuccessCondition : public BT::ConditionNode {
public:
    CheckingSuccessCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void updateTask();
    ros::Publisher task_alarm_;
    ros::Publisher send_checking_;
    ros::Publisher send_speaker_;
};
}  // namespace NaviFra
#endif// NC_CHECKING_SUCCESS_CONDITION_H
