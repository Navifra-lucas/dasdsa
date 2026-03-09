#ifndef NC_LOADING_SUCCESS_CONDITION_H
#define NC_LOADING_SUCCESS_CONDITION_H

namespace NaviFra {
class LoadingSuccessCondition : public BT::ConditionNode {
public:
    LoadingSuccessCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void updateTask();
    ros::Publisher task_alarm_;
    ros::Publisher send_loading_;
};
}  // namespace NaviFra
#endif// NC_LOADING_SUCCESS_CONDITION_H
