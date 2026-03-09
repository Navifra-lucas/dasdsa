#ifndef NC_MOVE_TO_LIFT_ACTION_H
#define NC_MOVE_TO_LIFT_ACTION_H

namespace NaviFra {
class MoveToLiftAction : public BT::SyncActionNode {
public:
    MoveToLiftAction(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher publiser_;
};
}  // namespace NaviFra
#endif