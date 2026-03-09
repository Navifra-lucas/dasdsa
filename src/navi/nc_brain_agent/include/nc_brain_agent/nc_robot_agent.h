#ifndef NC_ROBOTAGENT_H
#define NC_ROBOTAGENT_H

#include <Poco/Activity.h>
#include <Poco/Stopwatch.h>
#include <boost/format.hpp>
#include <core_agent/message/message_subscriber.h>
#include <core_agent/redis/redis_reader.h>
#include <nc_brain_agent/nc_robot_agent_initialize.h>
#include <nc_brain_agent/ros/nc_ros_launch_manager.h>
#include <nc_brain_agent/utils/nc_agent_config.h>

namespace NaviFra {

class NcRobotAgent : protected INcRobotAgentInitialize {
public:
    NcRobotAgent();
    ~NcRobotAgent();

    static NcRobotAgent& get();

    using Ptr = std::shared_ptr<NcRobotAgent>;

protected:
    void run();
    void runHearbeat();

public:
    bool initialize();
    void finalize();

public:
    void onMessage(const void*, MessageSubscriberArgs&);
    bool onRobotRequest(const std::string& message);
    bool onBackendStatus(const std::string& message);
    void onUpdatedMap();

    void initRoboCollision() override;
    void initActionManager() override;
    void initRosLaunchManager() override;
    void initRobotGroup() override;
    void initRobotStartUp() override;
    bool initMapInfo() override;
    bool initVerifyRobot() override;
    void initROS() override;

public:
    // runActivity 에서만 사용 하는 함수
    void start();
    void stop();
    bool isStopped();
    void initRobotCalibrationBase();
    void updateRobotInfo();

private:
    RedisReader::Ptr redisReader_;
    Poco::Activity<NcRobotAgent> activity_;

    ros::NodeHandle nodeHandler_;

    std::string robot_acs_reg_code_;
    std::string ip_address_;
};

inline bool NcRobotAgent::isStopped()
{
    return activity_.isStopped();
}

inline void NcRobotAgent::start()
{
    activity_.start();
}

inline void NcRobotAgent::stop()
{
    activity_.stop();
    activity_.wait(1000);
}
}  // namespace NaviFra

#endif  // NC_ROBOTAGENT_H