#ifndef NC_WAIT_SUCCESS_CONDITION_H
#define NC_WAIT_SUCCESS_CONDITION_H

namespace NaviFra {

class WaitSuccessCondition : public BT::ConditionNode {
public:
    WaitSuccessCondition(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

    std::chrono::system_clock::time_point tp_action_ = std::chrono::system_clock::now();

private:
    void resetInternalState();
    void updateTask();

    bool b_action_check_ = false;
    bool wsc_ = false;

    int wait_seconds_ = 0;
    std::string s_wait_id_;

    ros::Publisher task_alarm_;
};

}  // namespace NaviFra

#endif
