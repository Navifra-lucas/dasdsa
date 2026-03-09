#ifndef NC_ROBOT_INFO_H
#define NC_ROBOT_INFO_H

#include "core_msgs/MotorInfo.h"
#include "core_msgs/NavicoreStatus.h"

#include <Poco/Types.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nc_brain_agent/data/nc_agent_robot_collision_Info.h>
#include <nc_brain_agent/data/nc_entity.h>
#include <std_msgs/String.h>

#include <memory>

namespace NaviFra {

class NcRobotWheelInfoStore;
class NcRobotInfo : public NcEntity {
public:
    NcRobotInfo();
    ~NcRobotInfo();

    using Ptr = std::shared_ptr<NcRobotInfo>;

    const static std::string KEY;

public:
    void updateRobotInfo(const core_msgs::NavicoreStatus msg);
    void updateRobotStatus(const geometry_msgs::PoseWithCovarianceStamped msg);
    void updateRobotBase(const float wheel, const float offset);
    void updateVelocity(const double& linear_x, const double& linear_y, const double& angular);

    const RobotPose getPose() { return robot_pose_; }

    void updateJigInfo(const std_msgs::String::ConstPtr& msg);
    void setIsManual(bool isManual);

public:
    std::string getID() { return id_; };
    void setID(std::string id);
    void setSetting(int map, int totalNodes, int teachedNodes);
    void setTeachedNodes(int teachedNodes);
    void updateWheel(const core_msgs::MotorInfo::ConstPtr msg);

    void setStatus(std::string status);
    void setErrordist(float dist);
    void setRobotType(std::string type);
    void setBarcodeOn(bool bOnOff);
    void initCollision(float top, float bottom, float left, float right, float offsetX, float offsetY);
    void initShape(float top, float bottom, float left, float right, float offsetX, float offsetY);

    void setRobotDrivingInfo(bool isGoalStart, bool isGoalComplete, bool robotError, bool isMoving);
    void setMoveToGoal();
    void setGoalArrived();
    void setRobotBase(float f_base) { robot_wheelpos = f_base; }
    robotDrivingInfo getRobotDrivingInfo() { return robotDrivingInfo_; }
    std::string getRobotDrivingInfoToString();
    std::string getRobotType();

    std::string toString();
    std::string toString(std::shared_ptr<NcRobotCollisionInfo> collision, std::shared_ptr<NcRobotWheelInfoStore> wheelInfoStore);

private:
    virtual std::string implName() final { return "NcRobotInfo"; }

private:
    std::string id_;
    std::string robot_status_;
    std::string robot_alarm_;

    int battery_;
    RobotPose robot_pose_;

    double robot_linear_velocity_x_;
    double robot_linear_velocity_y_;
    double robot_angular_velocity_;

    std::string now_node_;
    std::string next_node_;
    std::string goal_node_;
    double path_progress_;
    double confidence_;

    std::string now_task_;
    std::string is_load_;
    RobotSize robot_size_;
    Setting setting_;

    Poco::FastMutex fastMutex_;

    float robot_wheelpos;
    float robot_wheeloffset;
    double error_dist;  // 주행 간 패스 오차

    Poco::JSON::Array robot_footprint_;
    Position lastPosition_;
    ErrorPose errorPose_;

    JigInfo jiginfo_;
    robotDrivingInfo robotDrivingInfo_;
    bool isrobotManual_;
    bool bardcode_reader_;

    std::vector<std::string> errortable_;  //임시
    std::string strRobottype_;
};
}  // namespace NaviFra

#endif  // NC_ROBOT_INFO_H