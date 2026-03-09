#ifndef NC_IS_DOCKING_CONDITION_H
#define NC_IS_DOCKING_CONDITION_H

namespace NaviFra {
class IsDockingCondition : public BT::ConditionNode {
public:
    IsDockingCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    ros::Publisher send_docking_;
    ros::Publisher send_docking_aruco_;
    ros::Publisher task_alarm_;
    ros::Publisher send_led_;
};
}  // namespace NaviFra
#endif  // NC_IS_DOCKING_CONDITION_H
