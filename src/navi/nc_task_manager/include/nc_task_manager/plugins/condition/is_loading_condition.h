#ifndef NC_IS_LOADING_CONDITION_H
#define NC_IS_LOADING_CONDITION_H

namespace NaviFra {
class IsLoadingCondition : public BT::ConditionNode {
public:
    IsLoadingCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher task_alarm_;
    ros::Publisher send_loading_;
};
}  // namespace NaviFra
#endif //NC_IS_LOADING_CONDITION_H
