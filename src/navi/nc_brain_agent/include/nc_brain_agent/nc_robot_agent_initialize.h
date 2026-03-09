#ifndef NC_ROBOT_AGENT_INITIALIZE_H
#define NC_ROBOT_AGENT_INITIALIZE_H
namespace NaviFra {
class INcRobotAgentInitialize {
public:
    virtual void initRoboCollision() = 0;
    virtual void initActionManager() = 0;
    virtual void initRosLaunchManager() = 0;
    virtual void initRobotGroup() = 0;
    virtual void initRobotStartUp() = 0;
    virtual bool initMapInfo() = 0;
    virtual bool initVerifyRobot() = 0;
    virtual void initROS() = 0;
};
}  // namespace NaviFra
#endif