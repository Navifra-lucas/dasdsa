#ifndef NAVIFRA_ROBOT_INFO_H
#define NAVIFRA_ROBOT_INFO_H

#include <Poco/JSON/Array.h>
#include <Poco/Mutex.h>
#include <Poco/Types.h>
#include <core_agent/data/robot_collision_info.h>
#include <core_agent/data/robot_pose.h>
#include <core_agent/data/robot_wheel_info.h>
#include <core_agent/data/types.h>

#include <memory>
#include <regex>
#include <vector>

namespace NaviFra {

// class RobotWheelInfoStore;
class RobotInfo {
public:
    RobotInfo();
    ~RobotInfo();

    using Ptr = std::shared_ptr<RobotInfo>;

    const static std::string KEY;

public:
    void updateRobotInfo(const Velocity& velocity, const NodePath& path, double confidence);
    void updateFootprint(const float& x, const float& y);
    void updateErrorPose(const float& x, const float& y, const float& deg);
    void updateJob(const std::string& current_task_id, const std::string& previous_task_id);
    // void updateRobotInfo(const core_msgs::NavicoreStatus msg);
    // void updateRobotStatus(const geometry_msgs::PoseWithCovarianceStamped msg);
    void updateRobotBase(const float wheel, const float offset);
    void updateVelocity(const double& linear_x, const double& linear_y, const double& angular);
    void updateTargetVelocity(const double& linear_x, const double& linear_y, const double& angular);

    // const RobotPose getPose() { return robot_pose_; }

    // void updateJigInfo(const std_msgs::String::ConstPtr& msg);
    void setIsManual(bool isManual);

public:
    std::string getID() { return id_; };
    void setID(std::string id);
    void setSetting(int map, int totalNodes, int teachedNodes);
    void setArea(std::string s_current_area_id, std::vector<std::string> vec_area_uuid, std::vector<std::string> vec_map_uuid);

    void setTeachedNodes(int teachedNodes);
    // void updateWheel(const core_msgs::MotorInfo::ConstPtr msg);

    void setStatus(std::string status);
    void setErrordist(float dist);
    void setRobotType(std::string type);
    void setBarcodeOn(bool bOnOff);
    void setBattery(float battery);
    void setBatteryVol(float battery_vol);
    void setForkUpDownCmd(int16_t fork_up_down_cmd);
    void setForkUpDownPosition(int16_t fork_up_down_position);
    void setTiltingUpDownCmd(int16_t tilting_up_down_cmd);
    void setForkWidthCmd(int16_t fork_width_cmd);

    void setRobotDrivingInfo(bool isGoalStart, bool isGoalComplete, bool robotError, bool isMoving);
    void setMoveToGoal();
    void setGoalArrived();
    void setRobotBase(float f_base) { robot_wheelpos = f_base; }
    robotDrivingInfo getRobotDrivingInfo() { return robotDrivingInfo_; }
    std::string getRobotDrivingInfoToString();
    std::string getRobotType();

    std::string getStatus() const { return robot_status_; }
    std::string getAlarmString() const { return robot_alarm_; }

    std::vector<float> getVelocity() const { return {robot_linear_velocity_x_, robot_linear_velocity_y_, robot_angular_velocity_}; }
    std::vector<float> getTargetVelocity() const { return {robot_linear_velocity_x_, robot_linear_velocity_y_, robot_angular_velocity_}; }

    std::string toString();
    std::string toString(
        std::shared_ptr<RobotCollisionInfo> collision, std::shared_ptr<RobotWheelInfoStore> wheelInfoStore,
        std::shared_ptr<RobotPose> pose);

    void setRobotPoseX(float robot_pose_x);
    void setRobotPoseY(float robot_pose_y);
    void setRobotPoseDeg(float robot_pose_deg);
    void setRobotLinearVelocityZ(float robot_linear_velocity_z);
    void setRobotPathSpeed(float f_max_linear_vel_of_path);
    void setAlarm(std::string alarm);
    void setMotorError(bool reset_cmd);
    void setBatteryCmd(int16_t batterystartcmd);
    void setLiftCmd(int16_t liftcmd);
    void setPioCmd(int64_t piocmd);
    void setChargingState(bool charging_state);
    void setAlarmDescription(std::string description);
    void setAlarmId(int16_t alarm_id);
    void setNowTask(std::string now_task);
    void setWorkID(std::string word_id);
    void setLight(bool light);
    void setOSSDField(int16_t ossd_field);
    void setForkLiftMax(int lift_max_dist);
    void setForkLiftMin(int lift_min_dist);
    void setPalletID(std::string pallet_id);

    std::string getStatus();

    int16_t getRobotPoseX();
    int16_t getRobotPoseY();
    int16_t getRobotPoseDeg();
    int16_t getConfidence();
    int16_t getNowNode();
    int16_t getNextNode();
    int16_t getGoalNode();
    int16_t getTwistLinearX();
    int16_t getTwistLinearZ();
    int16_t getRobotPathSpeed();
    int16_t getForkUpDownCmd();
    int16_t getForkUpDownPosition();
    int16_t getTiltingUpDownCmd();
    int16_t getForkWidthCmd();
    int16_t getAlarm();
    int16_t getMotorError();
    int16_t getBatteryCmd();
    int16_t getLiftCmd();
    int16_t getPioCmd();
    std::string getAlarmDescription();
    int16_t getAlarmId();
    bool getIsManual();
    int16_t getLight();
    int16_t getOSSDField();
    int16_t getForkLiftMax();
    int16_t getForkLiftMin();

    double getPathProgress() const { return path_progress_; }
    double getConfidence() const { return confidence_; }
    RobotSize getSize() const { return robot_size_; }

    std::string getNowNodeString() const { return now_node_; }
    std::string getNextNodeString() const { return next_node_; }
    std::string getGoalNodeString() const { return goal_node_; }

    float getBattery() const { return battery_; }
    float getBatteryVol() const { return battery_vol_; }
    double getErrordist() const { return error_dist; }

    std::string getIsLoad() const { return is_load_; }
    std::string getNowTask() const { return now_task_; }
    std::string getWorkID() const { return work_id_; }
    std::string getPalletID() const { return pallet_id_; }

    float getRobotBase() const { return robot_wheelpos; }
    float getRobotWheelOffset() const { return robot_wheeloffset; }
    Setting getSetting() const { return setting_; }

private:
    std::string id_;
    std::string robot_status_;
    std::string robot_alarm_;

    float robot_pose_x_;
    float robot_pose_y_;
    float robot_pose_deg_;
    float f_max_linear_vel_of_path_;

    int64_t liftcmd_;
    int64_t piocmd_;
    bool reset_cmd_;
    int16_t batterystartcmd_;

    std::string alarm_;

    std::string AlarmDescription_;
    int16_t Alarmid_;
    int16_t ossd_field_;
    std::string pallet_id_;

    float battery_;
    float battery_vol_;
    bool charging_state_;
    bool light_;
    int16_t lift_max_dist_;
    int16_t lift_min_dist_;

    float robot_linear_velocity_x_;
    float robot_linear_velocity_y_;
    float robot_linear_velocity_z_;
    float robot_angular_velocity_;
    float robot_target_linear_velocity_x_;
    float robot_target_linear_velocity_y_;
    float robot_target_angular_velocity_;

    std::string now_node_;
    std::string next_node_;
    std::string goal_node_;
    double path_progress_;
    double confidence_;

    std::string now_task_;
    std::string work_id_;
    std::string is_load_;
    RobotSize robot_size_;
    Setting setting_;
    Job tasks_;
    Map map_;

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
    Poco::FastMutex fastMutex_;

    int16_t fork_up_down_cmd_;
    int16_t fork_up_down_position_;
    int16_t tilting_up_down_cmd_;
    int16_t fork_width_cmd_;

    std::string s_current_area_id_;
    std::vector<std::string> vec_area_uuid_; 
    std::vector<std::string> vec_map_uuid_;
};
}  // namespace NaviFra

#endif  // NC_ROBOT_INFO_H