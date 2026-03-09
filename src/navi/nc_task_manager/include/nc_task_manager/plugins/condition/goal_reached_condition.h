#ifndef NC_GOAL_REACHED_CONDITION_H
#define NC_GOAL_REACHED_CONDITION_H

namespace NaviFra {
class GoalReachedCondition : public BT::ConditionNode {
public:
    GoalReachedCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void updateTask();
    void updateGoal();
    int sign(float val) { return (0 < val) - (val < 0); }

    float CalcAngleDomainRad_(float f_angle_rad)
    {
        return fmod((f_angle_rad + sign(f_angle_rad) * M_PI), 2.0 * M_PI) - sign(f_angle_rad) * M_PI;
    }

    float CalcAngleDomainDeg_(float f_angle_deg)
    {
        return fmod((f_angle_deg + sign(f_angle_deg) * 180.0), 360.0) - sign(f_angle_deg) * 180.0;
    }

private:
    ros::Publisher task_alarm_;
    ros::Publisher task_status_;
    ros::Publisher docking_state_pub_;

    move_msgs::CoreCommand goals_;
};
}  // namespace NaviFra
#endif