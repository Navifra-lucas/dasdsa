#ifndef NC_MOVE_TO_GOAL_ACTION_H
#define NC_MOVE_TO_GOAL_ACTION_H

namespace NaviFra {
class MoveToGoalAction : public BT::SyncActionNode {
public:
    MoveToGoalAction(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher sendGoal_;
    ros::Publisher sendLivePath_;
    ros::Publisher send_charging_;
    bool updated_goals;
};
}  // namespace NaviFra
#endif