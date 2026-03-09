#ifndef NC_IS_MOVE_CONDITION_H
#define NC_IS_MOVE_CONDITION_H

namespace NaviFra {
class IsMoveCondition : public BT::ConditionNode {
public:
    IsMoveCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher task_alarm_;
    ros::Publisher docking_state_pub_;
    void generatedGoal();
};
}  // namespace NaviFra
#endif