#include "nc_wia_agent/data/robot_basic_status.h"

using namespace NaviFra;

const std::string RobotBasicStatus::KEY = "RobotBasicStatus";

Poco::JSON::Object::Ptr RobotBasicStatus::toObject(const Pose2D& pose)
{
    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;
    obj->set("x", pose.x);
    obj->set("y", pose.y);
    obj->set("theta", pose.theta);
    obj->set("id", pose.id);
    return obj;
}

Poco::JSON::Object::Ptr RobotBasicStatus::toObject(const Vector3& vec)
{
    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;
    obj->set("x", vec.x);
    obj->set("y", vec.y);
    obj->set("z", vec.z);
    return obj;
}

Poco::JSON::Object::Ptr RobotBasicStatus::toObject(const Twist& twist)
{
    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;
    obj->set("linear", toObject(twist.linear));
    obj->set("angular", toObject(twist.angular));
    return obj;
}

Poco::JSON::Object::Ptr RobotBasicStatus::toObject() const
{
    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;

    obj->set("type", type_);
    obj->set("radius", radius_);
    obj->set("pose", toObject(pose_));
    obj->set("workstate", workstate_);
    obj->set("mode", mode_);
    obj->set("alarm", alarm_);
    obj->set("lccsstate", lccsstate_);
    obj->set("emergency", emergency_);
    obj->set("stand_by", stand_by_);
    obj->set("lift_status", lift_status_);
    obj->set("turntable_status", turntable_status_);
    obj->set("conv_status", conv_status_);
    obj->set("loaded", loaded_);
    obj->set("task_id", task_id_);
    obj->set("action_indx", action_indx_);
    obj->set("loop_count", loop_count_);
    obj->set("feed_vel", toObject(feed_vel_));
    obj->set("lift_", lift_);
    obj->set("lccs_onoff", lccs_onoff_);
    obj->set("lccs_stop_state", lccs_stop_state_);
    obj->set("lccs_level", lccs_level_);
    obj->set("lccs_manual_mode", lccs_manual_mode_);
    obj->set("turntable_", turntable_);
    obj->set("RFID", RFID_);
    obj->set("map_correction", map_correction_);
    obj->set("obstacle_state", obstacle_state_);
    obj->set("acceleration", toObject(acceleration_));
    obj->set("ir_state", ir_state_);
    obj->set("manual_charge", manual_charge_);
    obj->set("bms_volt_low", bms_volt_low_);
    obj->set("bms_volt_high", bms_volt_high_);
    obj->set("bms_current", bms_current_);
    obj->set("bms_temp_low", bms_temp_low_);
    obj->set("bms_temp_high", bms_temp_high_);

    // 배열 변환
    Poco::JSON::Array::Ptr driveArr = new Poco::JSON::Array;
    for (int val : drive_current_)
        driveArr->add(val);
    obj->set("drive_current", driveArr);

    obj->set("drive_temp", drive_temp_);
    obj->set("cbm_lift_volt", cbm_lift_volt_);
    obj->set("cbm_lift_current", cbm_lift_current_);
    obj->set("cbm_lift_torque", cbm_lift_torque_);
    obj->set("cbm_lift_temp", cbm_lift_temp_);
    obj->set("cbm_lift_rpm", cbm_lift_rpm_);
    obj->set("speed_limit_status", speed_limit_status_);
    obj->set("relay_state", relay_state_);
    obj->set("pause_reason", pause_reason_);
    obj->set("Cmd", Cmd_);
    obj->set("msg_time", msg_time_);
    obj->set("msg_id", msg_id_);
    obj->set("RID", RID_);
    obj->set("Result", Result_);
    obj->set("AmrId", RID_);

    return obj;
}

void RobotBasicStatus::setRobotID(std::string id)
{
    RID_ = id;
}

std::string RobotBasicStatus::getRobotID() const
{
    return RID_;
}

void RobotBasicStatus::setMsgID(int id)
{
    MsgID_ = id;
}

int RobotBasicStatus::getMsgID() const
{
    return MsgID_;
}

void RobotBasicStatus::setMode(int mode)
{
    mode_ = mode;
}

int RobotBasicStatus::getMode() const
{
    return mode_;
}

void RobotBasicStatus::setStandby(bool mode)
{
    stand_by_ = mode;
}

bool RobotBasicStatus::getStandby() const
{
    return stand_by_;
}

void RobotBasicStatus::setTaskID(std::string id)
{
    task_id_ = id;
}

void RobotBasicStatus::clearTaskID()
{
    task_id_.clear();
}

std::string RobotBasicStatus::getTaskID() const
{
    return task_id_;
}

void RobotBasicStatus::setLoopCnt(int loopcnt)
{
    LoopCnt_ = loopcnt;
}

int RobotBasicStatus::getLoopCnt() const
{
    return LoopCnt_;
}

void RobotBasicStatus::setGoalInfo(std::vector<GoalInfo> goalInfo)
{
    goal_info_ = goalInfo;
}

std::vector<GoalInfo> RobotBasicStatus::getGoalInfo() const
{
    return goal_info_;
}

void RobotBasicStatus::clearGoalInfo()
{
    goal_info_.clear();
}

void RobotBasicStatus::setWiaStatus(int status)
{
    wiaStatus_ = status;
}

int RobotBasicStatus::getWiaStatus() const
{
    return wiaStatus_;
}

void RobotBasicStatus::setWiaTask(int status)
{
    wiaTask_ = status;
}

int RobotBasicStatus::getWiaTask() const
{
    return wiaTask_;
}

void RobotBasicStatus::setAlarmDescription(std::string alarm)
{
    AlarmDescription_ = alarm;
}

std::string RobotBasicStatus::getAlarmDescription() const
{
    return AlarmDescription_;
}

void RobotBasicStatus::setAlarmId(int alarm_id)
{
    Alarmid_ = alarm_id;
}

int RobotBasicStatus::getAlarmId() const
{
    return Alarmid_;
}

void RobotBasicStatus::setEMS(bool status)
{
    emsON_ = status;
    setEmergency();
}

void RobotBasicStatus::setBumper(bool status)
{
    bumperOn_ = status;
    setEmergency();
}

void RobotBasicStatus::setEmergency()
{
    if (emsON_ && bumperOn_) {
        emergency_ = 3;
    }
    else if (!emsON_ && bumperOn_) {
        emergency_ = 2;
    }
    else if (emsON_ && !bumperOn_) {
        emergency_ = 1;
    }
    else {
        emergency_ = 0;
    }
}

int RobotBasicStatus::getEmergency() const
{
    if (emergency_ == -1) {
        return 0;
    }
    else {
        return emergency_;
    }
}

void RobotBasicStatus::setPlcVersion(std::string version)
{
    plcVersion_ = version;
}

std::vector<std::string> RobotBasicStatus::getVersion() const
{
    std::vector<std::string> arr;
    arr.emplace_back(workspaceVersion_);
    arr.emplace_back(subconVersion_);
    arr.emplace_back(plcVersion_);
    arr.emplace_back(rrsVersion_);
    return arr;
}

void RobotBasicStatus::setLiftStatus(int mode)
{
    lift_status_ = mode;
}

int RobotBasicStatus::getLiftStatus() const
{
    return lift_status_;
}

void RobotBasicStatus::setObstacleData(std::vector<float> data)
{
    obstacle_data_ = data;
}

std::vector<float> RobotBasicStatus::getObstacleData() const
{
    return obstacle_data_;
}

void RobotBasicStatus::setLidarData(std::vector<float> data)
{
    lidar_data_ = data;
}

std::vector<float> RobotBasicStatus::getLidarData() const
{
    return lidar_data_;
}

void RobotBasicStatus::setLastSubTime(Poco::Timestamp data)
{
    now_time_ = data;
}

Poco::Timestamp RobotBasicStatus::getLastSubTime() const
{
    return now_time_;
}

void RobotBasicStatus::setACSPause(bool status)
{
    acs_pause_ = status;
}

bool RobotBasicStatus::getACSPause() const
{
    return acs_pause_;
}

void RobotBasicStatus::setACSCancelCmd(bool cmd)
{
    acs_cancel_ = cmd;
}

bool RobotBasicStatus::getACSCancelCmd() const
{
    return acs_cancel_;
}

void RobotBasicStatus::setMqttConnect(bool status)
{
    mqtt_connect_ = status;
}

bool RobotBasicStatus::getMqttConnect() const
{
    return mqtt_connect_;
}

void RobotBasicStatus::setResultStopTrigger(bool status)
{
    result_stop_trigger_ = status;
}

bool RobotBasicStatus::getResultStopTrigger() const
{
    return result_stop_trigger_;
}

void RobotBasicStatus::setLoaded(int loaded)
{
    loaded_ = loaded;
}

int RobotBasicStatus::getLoaded() const
{
    return loaded_;
}

void RobotBasicStatus::setRFID(std::string RFID)
{
    RFID_ = RFID;
}

std::string RobotBasicStatus::getRFID() const
{
    return RFID_;
}

void RobotBasicStatus::setWingbodyOffset(const std::vector<float>& offset_values)
{
    wingbody_offset_ = offset_values;
}

std::vector<float> RobotBasicStatus::getWingbodyOffset() const
{
    return wingbody_offset_;
}

void RobotBasicStatus::Scenario1Active(bool msg)
{
    scenario_1_active_ = msg;
}

void RobotBasicStatus::Scenario2Active(bool msg)
{
    scenario_2_active_ = msg;
}

bool RobotBasicStatus::isScenario1Active() const
{
    return scenario_1_active_;
}

bool RobotBasicStatus::isScenario2Active() const
{
    return scenario_2_active_;
}