#ifndef NAVIFRA_ROBOT_BASIC_STATUS_H
#define NAVIFRA_ROBOT_BASIC_STATUS_H

#include "pos/pos.hpp"

#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>

#include <string>
#include <vector>

namespace NaviFra {

struct Vector3 {
    double x;
    double y;
    double z;
};

struct Pose2D {
    double x;
    double y;
    double theta;
    std::string id;
};

struct Twist {
    Vector3 linear;
    Vector3 angular;
};

struct GoalInfo {
    NaviFra::Pos o_pos;
    std::string action_type;
    int n_rack_type = -1;  // forklift action용, 다른 action은 -1
    int n_drive_type = -1;
};

class RobotBasicStatus {
public:
    const static std::string KEY;

    RobotBasicStatus() = default;
    ~RobotBasicStatus() = default;

    // JSON 변환
    Poco::JSON::Object::Ptr toObject() const;

    void setRobotID(std::string id);
    std::string getRobotID() const;

    void setMsgID(int id);
    int getMsgID() const;

    void setLoopCnt(int id);
    int getLoopCnt() const;

    void setTaskID(std::string id);
    std::string getTaskID() const;
    void clearTaskID();

    void setWiaTask(int count);
    int getWiaTask() const;

    void setWiaStatus(int status);
    int getWiaStatus() const;

    void setACSPause(bool status);
    bool getACSPause() const;

    void setACSCancelCmd(bool cmd);
    bool getACSCancelCmd() const;

    void setGoalInfo(std::vector<GoalInfo> goal_info);
    std::vector<GoalInfo> getGoalInfo() const;
    void clearGoalInfo();

    void setAlarmDescription(std::string status);
    std::string getAlarmDescription() const;

    void setAlarmId(int alarm_id);
    int getAlarmId() const;

    void setEMS(bool status);
    void setBumper(bool status);
    void setEmergency();
    int getEmergency() const;

    void setMode(int mode);
    int getMode() const;

    void setStandby(bool mode);
    bool getStandby() const;

    void setPlcVersion(std::string version);

    std::vector<std::string> getVersion() const;

    void setLiftStatus(int mode);
    int getLiftStatus() const;

    void setObstacleData(std::vector<float> data);
    std::vector<float> getObstacleData() const;

    void setLidarData(std::vector<float> data);
    std::vector<float> getLidarData() const;

    void setLastSubTime(Poco::Timestamp data);
    Poco::Timestamp getLastSubTime() const;

    void setMqttConnect(bool status);
    bool getMqttConnect() const;

    void setResultStopTrigger(bool status);
    bool getResultStopTrigger() const;

    void setLoaded(int loaded);
    int getLoaded() const;

    void setRFID(std::string RFID);
    std::string getRFID() const;

    void setWingbodyOffset(const std::vector<float>& offset_values);
    std::vector<float> getWingbodyOffset() const;

    void Scenario1Active(bool msg);
    void Scenario2Active(bool msg);

    bool isScenario1Active() const;
    bool isScenario2Active() const;

private:
    // 보조 변환 함수 (내부 static)
    static Poco::JSON::Object::Ptr toObject(const Pose2D& pose);
    static Poco::JSON::Object::Ptr toObject(const Vector3& vec);
    static Poco::JSON::Object::Ptr toObject(const Twist& twist);

private:
    std::string type_;
    float radius_;
    Pose2D pose_;
    int workstate_;
    int mode_;
    int alarm_;
    int lccsstate_;
    int emergency_ = -1;
    bool stand_by_ = false;
    int lift_status_;
    int turntable_status_;
    int conv_status_;
    int loaded_;
    bool acs_pause_ = false;
    bool acs_cancel_ = false;

    std::string task_id_;
    int action_indx_;
    int loop_count_;
    Twist feed_vel_;

    int lift_;
    int lccs_onoff_;
    int lccs_stop_state_;
    int lccs_level_;
    int lccs_manual_mode_;
    int turntable_;
    std::string RFID_;
    double map_correction_;
    int obstacle_state_;
    Twist acceleration_;

    int ir_state_;
    int manual_charge_;

    float bms_volt_low_;
    float bms_volt_high_;
    float bms_current_;
    float bms_temp_low_;
    float bms_temp_high_;

    std::vector<int> drive_current_;
    float drive_temp_;

    float cbm_lift_volt_;
    float cbm_lift_current_;
    float cbm_lift_torque_;
    float cbm_lift_temp_;
    float cbm_lift_rpm_;

    std::string speed_limit_status_;
    bool relay_state_;
    int pause_reason_;

    std::string Cmd_;
    std::string msg_time_;
    int msg_id_;
    std::string RID_;
    std::string Result_;
    std::string AmrId_;

    std::vector<GoalInfo> goal_info_;

    int MsgID_;
    int LoopCnt_;

    int wiaStatus_;
    int wiaTask_;

    std::string AlarmDescription_;
    int Alarmid_;

    bool emsON_ = false;
    bool bumperOn_ = false;

    std::string workspaceVersion_ = "";
    std::string subconVersion_ = "";
    std::string plcVersion_ = "";
    std::string rrsVersion_ = "";

    std::vector<float> obstacle_data_;
    std::vector<float> lidar_data_;

    Poco::Timestamp now_time_;

    bool mqtt_connect_;

    bool result_stop_trigger_;

    std::vector<float> wingbody_offset_ = {0, 0, 0};

    bool scenario_1_active_ = false;
    bool scenario_2_active_ = false;
};

}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_BASIC_STATUS_H
