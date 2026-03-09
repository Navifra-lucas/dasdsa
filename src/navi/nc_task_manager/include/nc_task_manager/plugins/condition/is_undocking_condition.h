#ifndef NC_IS_UNDOCKING_CONDITION_H
#define NC_IS_UNDOCKING_CONDITION_H

namespace NaviFra {
class IsUnDockingCondition : public BT::ConditionNode {
public:
    void generatedGoal();
    IsUnDockingCondition(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher sendLivePath_;
    ros::Publisher task_alarm_;
    ros::Publisher send_stopcharge_;
};
}  // namespace NaviFra
#endif  // NC_IS_UNDOCKING_CONDITION_H
