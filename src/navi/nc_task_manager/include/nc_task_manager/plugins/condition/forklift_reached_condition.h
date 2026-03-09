#ifndef NC_FORKLIFT_REACHED_CONDITION_H
#define NC_FORKLIFT_REACHED_CONDITION_H

namespace NaviFra {
class ForkLiftReachedCondition : public BT::ConditionNode {
public:
    ForkLiftReachedCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void updateTask();
    ros::Publisher task_alarm_;
};
}  // namespace NaviFra
#endif