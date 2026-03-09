#include "nc_wia_agent/initializer/statecheck_initializer.h"

#include "json.hpp"
#include "nc_wia_agent/util/plc_data_publisher.h"

#include <core_msgs/ForkLift.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace NaviFra;
namespace NaviFra {

//중간에 재시작됐을 때, 대응 로직 필요
bool StateCheckInitializer::isAtGoal(const GoalInfo& goal)
{
    auto robotPose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
    if (!robotPose)
        return false;

    float dist = std::hypot(robotPose->getPosition().x - goal.o_pos.GetXm(), robotPose->getPosition().y - goal.o_pos.GetYm());

    if (dist < 0.5) {
        NLOG(info) << "closest node position is : " << goal.o_pos.GetXm() << ", " << goal.o_pos.GetYm();
        NLOG(info) << "dist is : " << dist;
    }

    return dist < 0.2;
}

void StateCheckInitializer::handlePlcState(bool b_pinup)
{
    auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

    bool b_ems, b_bumper, b_automode;
    std::string s_navi_status = robotInfo->getStatus();

    b_ems = plc_info_.emergency;
    b_bumper = (plc_info_.front_bumper_switch || plc_info_.rear_bumper_switch) ? true : false;

    if (plc_info_.io_mode_select_1) {
        b_automode = true;
    }
    else if (plc_info_.io_mode_select_2) {
        b_automode = false;
    }

    if (plc_info_.front_up_px && plc_info_.rear_up_px) {
        robotStatus->setLiftStatus(1);
    }
    else if (b_pinup) {
        NLOG(info) << "pinup!!!!!!!";
        robotStatus->setLiftStatus(1);
    }
    else {
        robotStatus->setLiftStatus(2);
    }

    if (s_navi_status == "loading" || s_navi_status.empty()) {
        b_automode = false;
    }

    robotStatus->setEMS(b_ems);
    robotStatus->setBumper(b_bumper);
    robotStatus->setMode(b_automode);
    robotStatus->setPlcVersion(plc_info_.plc_version);
}

void StateCheckInitializer::cmdPub(std::string command)
{
    std_msgs::String msg;
    msg.data = command;
    navifra_cmd_pub.publish(msg);
}

void StateCheckInitializer::checkAcsHeartBeat()
{
    auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
    bool b_mqtt_connect = robotStatus->getMqttConnect();
    Poco::Timestamp lasttime = robotStatus->getLastSubTime();
    Poco::Timestamp now;

    // acs 시간초과 limit -> 1.5s
    bool b_acs_connect = (now - lasttime > 1500000) ? false : true;

    if (!b_mqtt_connect) {
        if (b_acs_connect_) {
            cmdPub("pause");
            b_acs_connect_ = false;
        }
        NLOG(info) << "state change, now mqtt notconnect";
        //에러처리 해야함
    }
    else if (!b_acs_connect) {
        if (b_acs_connect_) {
            cmdPub("pause");
            b_acs_connect_ = false;
            NLOG(info) << "state change, now acs notconnect";
        }
    }
    else if (!b_acs_connect_) {
        NLOG(info) << "state change, now acs connect";
        cmdPub("resume");
        b_acs_connect_ = true;
    }
}

void StateCheckInitializer::handleResult()
{
    auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
    bool b_mqtt_connect = robotStatus->getMqttConnect();

    if (!b_send_result_) {
        return;
    }

    if (!b_mqtt_connect || !b_acs_connect_) {
        return;
    }

    bool b_stop_send = robotStatus->getResultStopTrigger();
    NLOG(info) << "result stop trigger value : " << b_stop_send;
    if (b_stop_send) {
        robotStatus->setResultStopTrigger(false);
        robotStatus->clearGoalInfo();
        clearTaskText();
        b_send_result_ = false;
    }
    else {
        TaskResultPublisher::instance().publish("SUCCEEDED");
    }
}

void StateCheckInitializer::handleAlarm(std::string s_navi_status, std::string rid)
{
    auto alarmInfo = InMemoryRepository::instance().get<AlarmStatus>(AlarmStatus::KEY);

    if (!alarmInfo) {
        return;
    }

    if (n_start_time_) {
        Poco::Timestamp now;
        auto diff = now - *n_start_time_;
        if (diff < 10 * 1000000) {
            NLOG(info) << "return";
            return;  // 무시
        }
        n_start_time_.reset();
    }

    auto alarmList = alarmInfo->getAlarm();
    auto warningList = alarmInfo->getWarning();
    bool b_curr_alarm_exist = (alarmList.size() > 0);

    if (b_curr_alarm_exist) {
        NLOG(info) << "Publish alarm....";
        alarmInfo->PublishAlarm(rid);
    }
    else {
        if (b_last_alarm_exist_) {
            NLOG(info) << "All alarm cleared! Publish empty alarm";
            alarmInfo->PublishAlarm(rid);
        }
    }
    b_last_alarm_exist_ = b_curr_alarm_exist;

    if ((s_navi_status == "running" || s_navi_status == "idle") && AlarmManager::instance().size() > 0) {
        AlarmManager::instance().clearAllAlarms();
    }
}

void StateCheckInitializer::statusCheckCallback(const ros::TimerEvent& event)
{
    try {
        std_msgs::String str_msg;
        str_msg.data = "idle";

        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        if (!robotInfo || !robotStatus) {
            return;
        }

        int n_now_task = robotStatus->getWiaTask() + n_turntask_cnt_;
        std::string s_rid = robotStatus->getRobotID();
        std::string s_navi_status = robotInfo->getStatus();
        std::string s_prev_alarm_description = robotInfo->getAlarmDescription();

        int n_alarm_id = robotInfo->getAlarmId();
        std::vector<GoalInfo> goalList = robotStatus->getGoalInfo();
        int n_workstate = robotStatus->getWiaStatus();

        checkAcsHeartBeat();
        controlPlcScenario();

        handleAlarm(s_navi_status, s_rid);

        bool b_must_pinup = false;
        if (n_now_task == -1) {
            if (plc_info_.front_up_px || plc_info_.rear_up_px) {
                b_must_pinup = true;
            }
        }
        else if (n_now_task < goalList.size() && n_now_task >= 0) {
            const std::string& action = goalList[n_now_task].action_type;
            if (action != "move" && action != "turn") {
                if (plc_info_.front_up_px || plc_info_.rear_up_px) {
                    b_must_pinup = true;
                }
            }
        }
        handlePlcState(b_must_pinup);

        handleWingbodyOffset();

        if (goalList.empty()) {
            robotStatus->setWiaStatus(STATE_IDLE);
            robotStatus->setWiaTask(-1);
            b_send_result_ = false;
            b_fork_precontrol_sent_ = false;
            if (b_now_charge_) {
                NLOG(info) << "[CHARGE] NOW CHARGING.....";
                robotStatus->setWiaStatus(STATE_CHARGE);
                str_msg.data = "charging";
            }
            else {
                str_msg.data = "idle";
            }
            now_task_pub.publish(str_msg);
            return;
        }

        if (b_change_slam_) {
            if (goalList[n_now_task].action_type == "docking") {
                goalList[n_now_task].action_type = "slam_docking";
            }
        }

        if (n_workstate == STATE_PAUSE) {
            return;
        }
        if (n_now_task == -1) {
            str_msg.data = "idle";
            if (b_send_result_) {
                NLOG(info) << "handleResult called";
                handleResult();
            }
            else if (!goalList.empty()) {
                NLOG(info) << "[TASK] New task START! goal size is : " << goalList.size();
                b_task_done_ = false;
                n_turntask_cnt_ = 0;
                n_now_task = 0;
                robotStatus->setWiaTask(0);
            }
            else {
                robotStatus->setWiaStatus(STATE_IDLE);
            }
            now_task_pub.publish(str_msg);
            return;
        }

        if (n_now_task >= goalList.size()) {
            NLOG(info) << "[TASK] All task is done. switch to idle.";
            if (b_acs_connect_) {
                TaskResultPublisher::instance().publish("SUCCEEDED");
            }
            b_send_result_ = true;
            // result는 독립적으로 처리, 일단 IDLE로 상태 변환
            robotStatus->setWiaStatus(STATE_IDLE);
            robotStatus->setWiaTask(-1);
            n_turntask_cnt_ = 0;
            str_msg.data = "idle";
            b_change_slam_ = false;
            now_task_pub.publish(str_msg);
            return;
        }

        bool b_index_done = false;
        GoalInfo current_task = goalList[n_now_task];
        std::string s_curr_action_type = current_task.action_type;
        if (n_now_task != n_last_task_index_) {
            NLOG(info) << "current task is : " << s_curr_action_type << " / index is : " << n_now_task;
            NLOG(info) << "current task pos is : " << current_task.o_pos.GetXm() << " / " << current_task.o_pos.GetYm();
            NLOG(info) << "current turntask size is : " << n_turntask_cnt_;
            n_last_task_index_ = n_now_task;
        }

        if (s_curr_action_type == "standby") {
            robotStatus->setStandby(true);
        }
        else {
            robotStatus->setStandby(false);
        }

        bool b_is_last_task = (n_now_task + 1 >= goalList.size());
        if (s_curr_action_type == "move" && !b_is_last_task) {
            int forklift_index = -1;
            if (checkForklift(goalList, n_now_task, forklift_index)) {
                float f_remaining_dist = calculateDistance(goalList, n_now_task, forklift_index);
                // Threshold 이내이고, 아직 명령을 보내지 않았으면 전송
                if (f_remaining_dist < f_fork_precontrol_threshold_ && !b_fork_precontrol_sent_) {
                    const GoalInfo& forklift_goal = goalList[forklift_index];
                    publishForkPreControl(forklift_goal.n_rack_type, forklift_goal.n_drive_type);
                    b_fork_precontrol_sent_ = true;  // 중복 전송 방지
                    NLOG(info) << "[Fork PreControl] Sent command - Distance: " << f_remaining_dist
                               << "m, rack_type: " << forklift_goal.n_rack_type << ", drive_type: " << forklift_goal.n_drive_type;
                }
            }
            else {
                b_fork_precontrol_sent_ = false;
            }

            GoalInfo next_task = goalList[n_now_task + 1];
            if (next_task.action_type == "move") {
                if (isAtGoal(current_task)) {
                    b_index_done = true;
                    b_task_done_ = false;
                }
            }
            else {
                if (b_task_done_) {
                    b_index_done = true;
                    b_task_done_ = false;
                }
            }
        }
        else {
            if (b_task_done_) {
                b_index_done = true;
                b_task_done_ = false;
                if (s_curr_action_type == "turn") {
                    n_turntask_cnt_++;
                }
            }
        }

        if (b_index_done) {
            n_now_task++;
            b_change_slam_ = false;
            updateIndexText(n_now_task);
            int n_wia_task_cnt = n_now_task - n_turntask_cnt_;
            robotStatus->setWiaTask(n_wia_task_cnt);
            NLOG(info) << "[TASK]" << current_task.action_type << " task " << n_now_task - 1 << " completed. index changed -> "
                       << n_now_task;
        }

        if (n_now_task < goalList.size()) {
            GoalInfo tmp_task = goalList[n_now_task];
            if (tmp_task.action_type == "move") {
                robotStatus->setWiaStatus(STATE_MOVE);
            }
            else if (
                tmp_task.action_type == "docking" || tmp_task.action_type == "undocking" || tmp_task.action_type == "standby" ||
                tmp_task.action_type == "forklift" || tmp_task.action_type == "slam_docking" || tmp_task.action_type == "lift" ||
                tmp_task.action_type == "charge_docking") {
                robotStatus->setWiaStatus(STATE_EVENT);
            }

            str_msg.data = tmp_task.action_type;
        }
        // charge, pause, resume 제외
        now_task_pub.publish(str_msg);
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::handleWingbodyOffset()
{
    try {
        Poco::FastMutex::ScopedLock lock(wingbodyOffsetMutex_);
        if (wingbody_offset_checker_.isNeedCompare()) {
            if (!wingbody_offset_checker_.isTimeout()) {
                if (wingbody_offset_checker_.compare()) {
                    wingbody_offset_checker_.reset();
                    std_msgs::Bool check_msg;
                    check_msg.data = true;
                    wingbody_check_pub_.publish(check_msg);
                }
                else {
                    float x, y, theta;

                    wingbody_offset_checker_.getData(x, y, theta);

                    auto& plcPub = NaviFra::PlcTaskPublisher::instance();
                    plcPub.publishOffset("offset_x", x);
                    plcPub.publishOffset("offset_y", y);
                    plcPub.publishOffset("offset_theta", theta);
                }
            }
            else {
                NLOG(info) << "[WingbodyOffset] Timeout!!!!!!!!";
                //오류 처리 하기
            }
        }
    }
    catch (const std::exception& e) {
        NLOG(error) << "[HandleWingbodyOffset EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::RecvNaviCmd(const std_msgs::String::ConstPtr& msg)
{
    static int n_wia_status = 0;
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        std::string s_cmd = msg->data;
        if (s_cmd == "cancel") {
            robotStatus->setWiaTask(-1);
            robotStatus->setWiaStatus(STATE_IDLE);
            robotStatus->clearGoalInfo();
            NLOG(info) << "Subscribe Navifra/cmd cancel";
            TaskResultPublisher::instance().publish("PREEMPTED");

            // 자동 미션 중, 임의 캔슬 발생 시 오류 발행. 일단 undefined
            bool b_acs_cancel = robotStatus->getACSCancelCmd();
            if (!b_acs_cancel) {
                std::string s_rid = robotStatus->getRobotID();
                try {
                    Poco::JSON::Array::Ptr data = new Poco::JSON::Array;
                    data->add(static_cast<uint64_t>(WiaAlarm::ERROR_UNDEFINED));

                    int msg_id = 1;
                    Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
                    response->set("Cmd", "error");
                    response->set("data", data);
                    response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
                    response->set("msg_id", msg_id);
                    response->set("RID", s_rid);
                    response->set("Result", "S");
                    response->set("AmrId", s_rid);

                    std::ostringstream oss;
                    response->stringify(oss);

                    MessageBroker::instance().publish(s_rid + ".ACS", oss.str());
                    NLOG(warning) << "[CANCEL_ERROR] Cancel occurred without ACS command. Error sent to ACS: " << oss.str();
                }
                catch (const Poco::Exception& ex) {
                    NLOG(error) << "[CANCEL_ERROR] Failed to send error to ACS: " << ex.displayText();
                }
            }
        }
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::RecvNaviAlarm(const core_msgs::NaviAlarm::ConstPtr& msg)
{
    try {
        if (msg->alarm >= 2000) {
            auto alarmInfo = InMemoryRepository::instance().get<AlarmStatus>(AlarmStatus::KEY);
            alarmInfo->setAlarm(msg->alarm);
        }
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

//충전도킹 충전 거리 기준치초과일 때 알람 ㄱ

void StateCheckInitializer::RecvTaskAlarm(const core_msgs::TaskAlarm::ConstPtr& msg)
{
    try {
        if (msg->alarm == "done") {
            NLOG(info) << "Subscribe Task_manager task done alarm";
            b_task_done_ = true;
        }
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::updateAlarmFromStatus()
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        auto alarmInfo = InMemoryRepository::instance().get<AlarmStatus>(AlarmStatus::KEY);
        Alarms alarmlist = alarmInfo->getAlarm();

        std::string s_status = robotInfo->getStatus();

        if (n_obstacle_time_.isElapsed(0)) {
            n_obstacle_time_.update();
        }

        if (s_prev_status_.empty()) {
            s_prev_status_ = s_status;
        }
        else {
            if (alarmlist.size() > 0) {
                if (s_status == "running" || s_status == "paused") {
                    // current alarm 과 previous alarm 을 clear 하고 paused_by_obstacle time 을 update
                    if (s_prev_status_ == "paused_by_obs_camera")
                        alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_CAMERA_OBSTACLE);
                    else if (s_prev_status_ == "paused_by_obs_v2v")
                        alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_V2V_OBSTACLE);
                    else if (s_prev_status_ == "paused_by_obs_lidar")
                        alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_LIDAR_OBSTACLE);
                    else if (s_prev_status_ == "paused_by_dock_out_t_zone_obstacle")
                        alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_T_ZONE_OBSTACLE);
                    else
                        alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSED_BY_OBSTACLE);
                }
                n_obstacle_time_.update();
            }

            if (s_status == "idle") {
                if (s_prev_status_ == "ERROR_PATH_NOT_CREATED")
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PATH_NOT_CREATED);
                else if (s_prev_status_ == "paused_by_obs_camera")
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_CAMERA_OBSTACLE);
                else if (s_prev_status_ == "paused_by_obs_v2v")
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_V2V_OBSTACLE);
                else if (s_prev_status_ == "paused_by_obs_lidar")
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_LIDAR_OBSTACLE);
                else if (s_prev_status_ == "paused_by_dock_out_t_zone_obstacle")
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_T_ZONE_OBSTACLE);
                else
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSED_BY_OBSTACLE);
                n_obstacle_time_.update();
            }
            else {
                if (s_status == "ERROR_PATH_OUT") {
                    alarmInfo->setAlarm("ERROR_PATH_OUT");
                    // prev_alarm_status_ = cur_alarm_status_;
                    n_obstacle_time_.update();
                }
                else if (s_status == "paused_by_obs_camera") {
                    alarmInfo->setAlarm("paused_by_obs_camera");
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_V2V_OBSTACLE);
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_LIDAR_OBSTACLE);
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_T_ZONE_OBSTACLE);
                }
                else if (s_status == "paused_by_obs_v2v") {
                    alarmInfo->setAlarm("paused_by_obs_v2v");
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_CAMERA_OBSTACLE);
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_LIDAR_OBSTACLE);
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_T_ZONE_OBSTACLE);
                }
                else if (s_status == "paused_by_obs_lidar") {
                    alarmInfo->setAlarm("paused_by_obs_lidar");
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_V2V_OBSTACLE);
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_CAMERA_OBSTACLE);
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_T_ZONE_OBSTACLE);
                }
                else if (s_status == "paused_by_dock_out_t_zone_obstacle") {
                    alarmInfo->setAlarm("paused_by_dock_out_t_zone_obstacle");
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_V2V_OBSTACLE);
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_LIDAR_OBSTACLE);
                    alarmInfo->clearSpecificAlarm(WiaAlarm::ERROR_PAUSE_BY_CAMERA_OBSTACLE);
                }
                // prev_alarm_status_ = cur_alarm_status_;
                Poco::Timestamp now;
                if ((now - n_obstacle_time_) / 1000 >= n_paused_by_obstacle_timeout_ * 1000)  // 1s
                {
                    alarmInfo->setAlarm("paused_by_obs_timeout");
                }
            }
            s_prev_status_ = s_status;
        }
    }  // namespace NaviFra
    catch (Poco::BadCastException& ex) {
        NLOG(error) << ex.displayText();
    }
}

void StateCheckInitializer::alarmCheckCallback(const ros::TimerEvent& e)
{
    try {
        updateAlarmFromStatus();

        auto alarmInfo = InMemoryRepository::instance().get<AlarmStatus>(AlarmStatus::KEY);
        // 현재 시간
        Poco::Timestamp now;

        Alarms alarmlist = alarmInfo->getAlarm();

        for (const auto& alarm : alarmlist) {
            auto name = alarm.first;
            const Poco::Timestamp& alarmTime = alarm.second;

            // if (alarmInfo->IsStringInVolatileNaviAlarm(name)) {
            int64_t elapsed_ms = (now - alarmTime) / 1000;

            NLOG(info) << boost::format("event alarm: %1% , elapse: %2%") % name % elapsed_ms;

            if (elapsed_ms > 3000) {
                try {
                    alarmInfo->clearSpecificAlarm(name);
                    NLOG(info) << "auto clear " << name;

                    auto alarm_enum = alarm.first;
                }
                catch (const std::invalid_argument& e) {
                    NLOG(error) << "invalid alarm status: " << e.what();
                }
            }
            // }
        }
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "alarmCheckCallback failed: " << e.displayText();
    }
}

void StateCheckInitializer::pubchargerelay(bool state)
{
    try {
        auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        std::string rid = status->getRobotID();
        int msg_id = 1;

        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "charge_relay");
        response->set("data", state);
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(rid + ".ACS", oss.str());
        NLOG(info) << "Published Alarm Message: " << oss.str();  // 테스트용 로그 출력
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Alarm Exception: " << ex.displayText();
    }
}

void StateCheckInitializer::RecvPlcInfo(const core_msgs::PLCInfo::ConstPtr& msg)
{
    try {
        Poco::FastMutex::ScopedLock lock(plcMutex_);
        plc_info_ = *msg;
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}  // namespace NaviFra

void StateCheckInitializer::RecvBmsInfo(const core_msgs::BmsInfo::ConstPtr& msg)
{
    try {
        auto battery_status = InMemoryRepository::instance().get<BatteryStatus>(BatteryStatus::KEY);
        battery_status->setCapacity(msg->f32_soc);
        battery_status->setVoltage(msg->f32_pack_volt);
        battery_status->setCurrent(msg->f32_pack_current);
        battery_status->setEqCycle(0);
        battery_status->setPackOvervoltage(0);
        battery_status->setPackUndervoltage(0);
        battery_status->setCellOvervoltage(0);
        battery_status->setCellUndervoltage(0);
        battery_status->setOverTemperature(0);
        battery_status->setUnderTemperature(0);
        battery_status->setCellVoltageDiffFault(0);
        battery_status->setCellTempDiffFault(0);
        battery_status->setOvercurrent(0);
        battery_status->setChargeCompleteSignal(msg->b_charge_complete);
        battery_status->setChargingSignal(msg->b_charging);
        battery_status->setCellMaxVoltage(msg->f32_max_cell_v);
        battery_status->setCellMinVoltage(msg->f32_min_cell_v);
        battery_status->setCellAvgVoltage(msg->f32_avg_cell_v);
        battery_status->setCellMaxVoltagePos(msg->un8_max_v_cell_no);
        battery_status->setCellMinVoltagePos(msg->un8_min_v_cell_no);
        battery_status->setCellMaxTemp(msg->f32_max_temp);
        battery_status->setCellMinTemp(msg->f32_min_temp);
        battery_status->setCellAvgTemp(msg->f32_avg_temp);
        battery_status->setMaxTempPos(msg->un8_max_temp_pos);
        battery_status->setMinTempPos(msg->un8_min_temp_pos);
        battery_status->setPeekVoltage(0);
        battery_status->setPeekCurrent(0);
        battery_status->setPeekTemperature(0);
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}
void StateCheckInitializer::RecvCheonilInfo(const core_msgs::CheonilReadRegister::ConstPtr& msg)
{
    auto battery_status = InMemoryRepository::instance().get<BatteryStatus>(BatteryStatus::KEY);
    auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

    battery_status->setCapacity(msg->battery);
    battery_status->setChargingSignal(msg->charge_state);
    int n_loaded = 0;
    if (msg->load_state > 0 || msg->pallet_touch == 1) {
        n_loaded = 1;
    }
    robotStatus->setLoaded(n_loaded);

    bool new_charge_state = static_cast<bool>(msg->charge_state);

    // 상태 변환 시 charge on, relay publish
    if (new_charge_state != b_now_charge_) {
        b_now_charge_ = new_charge_state;

        pubchargerelay(b_now_charge_);

        NLOG(info) << "[StateCheck] Charge state changed → " << (b_now_charge_ ? "CHARGING" : "NOT CHARGING");
    }
}

void StateCheckInitializer::RecvPalletID(const std_msgs::String::ConstPtr& msg)
{
    auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
    std::string s_pallet_id = msg->data;
    robotStatus->setRFID(s_pallet_id);
}

void StateCheckInitializer::RecvMissionRestart(const std_msgs::String::ConstPtr& msg)
{
    try {
        std::string s_cmd = msg->data;
        if (s_cmd == "restart") {
            if (checkTaskTxt()) {
                restartTextTask();
            }
        }
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::RecvObs(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        if (!robotStatus)
            return;

        const int num_bins = 3600;
        std::vector<float> vec_obs(num_bins, 51);

        for (const auto& p : msg->polygon.points) {
            auto r = hypot(p.x, p.y);
            auto deg = std::atan2(p.y, p.x) * 180.0f / M_PI;
            if (deg < 0) {
                deg += 360.0f;
            }
            deg += 180;
            if (deg >= 360) {
                deg -= 360;
            }

            int idx = static_cast<int>(deg * 10.0f);
            if (idx >= 0 && idx < num_bins)
                if (r < vec_obs[idx]) {
                    vec_obs[idx] = r;
                }
        }
        robotStatus->setObstacleData(vec_obs);
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::RecvScan(const sensor_msgs::PointCloudConstPtr& msg)
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        if (!robotStatus)
            return;
        const int num_bins = 3600;
        std::vector<float> vec_lidar(num_bins, 51);

        for (const auto& p : msg->points) {
            auto r = hypot(p.x, p.y);
            auto deg = std::atan2(p.y, p.x) * 180.0f / M_PI;
            if (deg < 0) {
                deg += 360.0f;
            }
            deg += 180;
            if (deg >= 360) {
                deg -= 360;
            }

            int idx = static_cast<int>(deg * 10.0f);
            if (idx >= 0 && idx < num_bins)
                if (r < vec_lidar[idx]) {
                    vec_lidar[idx] = r;
                }
        }
        robotStatus->setLidarData(vec_lidar);
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::RecvSlamTrigger(const std_msgs::Bool::ConstPtr& trigger)
{
    try {
        if (trigger->data) {
            b_change_slam_ = true;
        }
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::RecvWingBodyPos(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    try {
        Poco::FastMutex::ScopedLock lock(wingbodyMutex_);
        wingbody_poses_ = *msg;
        NLOG(info) << "[StateCheck] Wingbody poses received: " << msg->poses.size() << " poses";
    }
    catch (std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::RecvWingBodyTrigger(const std_msgs::Int32::ConstPtr& msg)
{
    try {
        int index = msg->data;

        geometry_msgs::PoseArray poses;
        {
            Poco::FastMutex::ScopedLock lock(wingbodyMutex_);
            poses = wingbody_poses_;
        }

        if (poses.poses.empty()) {
            NLOG(error) << "[StateCheck] No wingbody poses available!";
            return;
        }

        if (index < 0 || index >= poses.poses.size()) {
            NLOG(error) << "[StateCheck] Invalid trigger index: " << index << " (available: 0-" << poses.poses.size() - 1 << ")";
            return;
        }

        // Get robot current position
        auto robotPose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
        if (!robotPose) {
            NLOG(error) << "[StateCheck] Robot pose not available!";
            return;
        }

        // Get target pose from array
        const auto& target_pose = poses.poses[index];

        // Convert quaternion to yaw
        tf2::Quaternion q(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double target_deg = yaw * (180.0 / M_PI);

        // Create and publish ForkLift message
        core_msgs::ForkLift fork_lift;
        fork_lift.s_current_node_id = "";
        fork_lift.s_target_node_id = "";
        fork_lift.f_current_x = robotPose->getPosition().x;
        fork_lift.f_current_y = robotPose->getPosition().y;
        fork_lift.f_current_deg = target_deg;
        fork_lift.f_target_x = target_pose.position.x;
        fork_lift.f_target_y = target_pose.position.y;
        fork_lift.f_target_deg = target_deg;
        fork_lift.n_rack_level = 0;
        fork_lift.n_target_level = 0;
        fork_lift.n_target_height = 0;
        fork_lift.n_drive_type = 3;  // As specified
        fork_lift.n_rack_type = 2;  // As specified
        fork_lift.n_pallet_type = 0;

        fork_docking_pub_.publish(fork_lift);

        NLOG(info) << "[StateCheck] Wingbody trigger " << index << " -> target: (" << fork_lift.f_target_x << ", " << fork_lift.f_target_y
                   << ", " << fork_lift.f_target_deg << "°)";
    }
    catch (std::exception& e) {
        NLOG(error) << "[EXCEPTION] RecvWingBodyTrigger: " << e.what();
    }
}

void StateCheckInitializer::RecvWingBodyApproachMoveTrigger(const std_msgs::Int32::ConstPtr& msg)
{
    try {
        int index = msg->data;
        // Convert PoseArray to CoreCommand and forward to /navifra/live_path
        move_msgs::CoreCommand tmp_path;
        float f_speed = 0.4;
        auto robotPose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
        geometry_msgs::PoseArray poses;
        {
            Poco::FastMutex::ScopedLock lock(wingbodyMutex_);
            poses = wingbody_poses_;
        }
        const auto& target_pose = poses.poses[index];

        for (int i = 0; i < 2; i++) {
            move_msgs::Waypoint wp;

            // Extract position
            if (i == 0) {
                wp.f_x_m = robotPose->getPosition().x;
                wp.f_y_m = robotPose->getPosition().y;
            }
            else if (i == 1) {
                // Convert quaternion to yaw angle in degrees
                tf2::Quaternion q(
                    target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                wp.f_angle_deg = yaw * (180.0 / M_PI);

                wp.f_x_m = target_pose.position.x + 3.5 * cos(yaw);
                wp.f_y_m = target_pose.position.y + 3.5 * sin(yaw);
            }

            wp.f_speed_ms = f_speed;

            wp.n_drive_type = 2000;
            wp.s_id = "";
            wp.s_name = "";
            wp.list_f_move_obstacle_margin.clear();
            wp.b_stop_quick = false;
            wp.b_start_quick = true;

            move_msgs::LidarObstacle lidar_obs;
            lidar_obs.list_f_obstacle_margin.resize(4);
            lidar_obs.list_f_obstacle_margin.at(0) = 0.5;
            lidar_obs.list_f_obstacle_margin.at(1) = 0.5;
            lidar_obs.list_f_obstacle_margin.at(2) = 0.5;
            lidar_obs.list_f_obstacle_margin.at(3) = 0.5;

            tmp_path.list_waypoints.push_back(wp);
            tmp_path.list_lidar_obs.push_back(lidar_obs);
        }

        live_path_pub.publish(tmp_path);
    }
    catch (std::exception& e) {
        NLOG(error) << "[EXCEPTION] RecvWingBodyApproachMoveTrigger: " << e.what();
    }
}

void StateCheckInitializer::RecvHplcTest(const geometry_msgs::Pose::ConstPtr& msg)
{
    try {
        auto& plcPub = NaviFra::PlcTaskPublisher::instance();

        // Convert quaternion to yaw
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // double d_deg = yaw * (180.0 / M_PI);

        NLOG(info) << "[HPLC] Publishing wingbody offset: x=" << msg->position.x << ", y=" << msg->position.y << ", theta=" << yaw;

        plcPub.publishOffset("offset_x", msg->position.x);
        plcPub.publishOffset("offset_y", msg->position.y);
        plcPub.publishOffset("offset_theta", yaw);

        {
            Poco::FastMutex::ScopedLock lock(wingbodyOffsetMutex_);
            wingbody_offset_checker_.update(msg->position.x, msg->position.y, yaw);
        }
    }
    catch (const std::exception& e) {
        NLOG(info) << "[EXCEPTION] " << e.what();
    }
}

void StateCheckInitializer::controlPlcScenario()
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        auto b_scenario_1_active = robotStatus->isScenario1Active();
        auto b_scenario_2_active = robotStatus->isScenario2Active();
        NLOG(info) << "Scenario 1 active : " << b_scenario_1_active << " / state : " << scenario_1_state_;
        NLOG(info) << "Scenario 2 active : " << b_scenario_2_active << " / state : " << scenario_2_state_;
        
        switch (scenario_1_state_) {
            case SCENARIO_IDLE:
                if (b_scenario_1_active) {
                    NLOG(info) << "[ScenarioTrigger] Scenario 1 is active. Sending trigger.";
                    std_msgs::Int32 data;
                    data.data = 1;
                    scenario_pub_.publish(data);
                    scenario_1_state_ = SCENARIO_TRIGGERED;
                }
                break;

            case SCENARIO_TRIGGERED:
                break;

            case SCENARIO_WAITING_RESET:
                if (!b_scenario_1_active) {
                    NLOG(info) << "[ScenarioTrigger] Scenario 1 PLC reset confirmed. Back to IDLE.";
                    scenario_1_state_ = SCENARIO_IDLE;
                    scenario_1_retry_count_ = 0;  // 카운터 리셋
                    // 완료 플래그도 리셋 (안전을 위해)
                    scenario_1_1_completed_ = false;
                    scenario_1_2_completed_ = false;
                }
                else {
                    scenario_1_retry_count_++;
                    if (scenario_1_retry_count_ >= SCENARIO_RETRY_THRESHOLD) {
                        NLOG(warning) << "[ScenarioTrigger] Scenario 1 PLC reset timeout (" << scenario_1_retry_count_
                                      << " checks). Retrying PLC write for both 1 and 2.";
                        auto& plcPub = NaviFra::PlcTaskPublisher::instance();
                        plcPub.controlScenario(0, 1);  // 1-1 다시 끄기
                        plcPub.controlScenario(0, 2);  // 1-2 다시 끄기
                        scenario_1_retry_count_ = 0;  // 카운터 리셋 후 재시도
                    }
                }
                break;
        }

        // Scenario 2 state machine
        switch (scenario_2_state_) {
            case SCENARIO_IDLE:
                if (b_scenario_2_active) {
                    NLOG(info) << "[ScenarioTrigger] Scenario 2 is active. Sending trigger.";
                    std_msgs::Int32 data;
                    data.data = 2;
                    scenario_pub_.publish(data);
                    scenario_2_state_ = SCENARIO_TRIGGERED;
                }
                break;

            case SCENARIO_TRIGGERED:
                break;

            case SCENARIO_WAITING_RESET:
                if (!b_scenario_2_active) {
                    NLOG(info) << "[ScenarioTrigger] Scenario 2 PLC reset confirmed. Back to IDLE.";
                    scenario_2_state_ = SCENARIO_IDLE;
                    scenario_2_retry_count_ = 0;  // 카운터 리셋
                    // 완료 플래그도 리셋 (안전을 위해)
                    scenario_2_1_completed_ = false;
                    scenario_2_2_completed_ = false;
                }
                else {
                    scenario_2_retry_count_++;
                    if (scenario_2_retry_count_ >= SCENARIO_RETRY_THRESHOLD) {
                        NLOG(warning) << "[ScenarioTrigger] Scenario 2 PLC reset timeout (" << scenario_2_retry_count_
                                      << " checks). Retrying PLC write for both 3 and 4.";
                        auto& plcPub = NaviFra::PlcTaskPublisher::instance();
                        plcPub.controlScenario(0, 3);  // 2-1 다시 끄기
                        plcPub.controlScenario(0, 4);  // 2-2 다시 끄기
                        scenario_2_retry_count_ = 0;  // 카운터 리셋 후 재시도
                    }
                }
                break;
        }
    }
    catch (std::exception& e) {
        NLOG(error) << "[EXCEPTION] controlPlcScenario: " << e.what();
    }
}

void StateCheckInitializer::RecvScenarioTrigger(const std_msgs::Int32::ConstPtr& msg)
{
    try {
        auto& plcPub = NaviFra::PlcTaskPublisher::instance();
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        int n_scenario_index = msg->data;
        NLOG(info) << "[ScenarioTrigger] Received scenario complete signal: " << n_scenario_index;

        // Handle "all operations complete" signal
        if (n_scenario_index == 100) {
            NLOG(info) << "[ScenarioTrigger] Received all operations complete signal. Resetting all scenarios.";
            resetAllScenarios();
            return;
        }

        auto b_scenario_1_active = robotStatus->isScenario1Active();
        auto b_scenario_2_active = robotStatus->isScenario2Active();

        // 각 서브 시나리오마다 개별적으로 PLC write
        if (n_scenario_index == 1) {
            NLOG(info) << "[ScenarioTrigger] Scenario 1-1 completed. Writing PLC to turn off 1.";
            plcPub.controlScenario(0, 1);
            scenario_1_1_completed_ = true;
        }
        else if (n_scenario_index == 2) {
            NLOG(info) << "[ScenarioTrigger] Scenario 1-2 completed. Writing PLC to turn off 2.";
            plcPub.controlScenario(0, 2);
            scenario_1_2_completed_ = true;
        }
        else if (n_scenario_index == 3) {
            NLOG(info) << "[ScenarioTrigger] Scenario 2-1 completed. Writing PLC to turn off 3.";
            plcPub.controlScenario(0, 3);
            scenario_2_1_completed_ = true;
        }
        else if (n_scenario_index == 4) {
            NLOG(info) << "[ScenarioTrigger] Scenario 2-2 completed. Writing PLC to turn off 4.";
            plcPub.controlScenario(0, 4);
            scenario_2_2_completed_ = true;
        }

        // Scenario 1: 1-1과 1-2 둘 다 완료되었는지 확인 → WAITING_RESET 상태로 전환
        if (scenario_1_1_completed_ && scenario_1_2_completed_ && b_scenario_1_active) {
            NLOG(info) << "[ScenarioTrigger] All Scenario 1 sub-scenarios completed. Waiting for PLC reset confirmation.";
            scenario_1_state_ = SCENARIO_WAITING_RESET;
            scenario_1_retry_count_ = 0;
        }

        // Scenario 2: 2-1과 2-2 둘 다 완료되었는지 확인 → WAITING_RESET 상태로 전환
        if (scenario_2_1_completed_ && scenario_2_2_completed_ && b_scenario_2_active) {
            NLOG(info) << "[ScenarioTrigger] All Scenario 2 sub-scenarios completed. Waiting for PLC reset confirmation.";
            scenario_2_state_ = SCENARIO_WAITING_RESET;
            scenario_2_retry_count_ = 0;
        }
    }
    catch (std::exception& e) {
        NLOG(error) << "[EXCEPTION] RecvScenarioTrigger: " << e.what();
    }
}

void StateCheckInitializer::resetAllScenarios()
{
    try {
        auto& plcPub = NaviFra::PlcTaskPublisher::instance();
        
        NLOG(info) << "[ScenarioReset] All operations completed. Resetting all scenarios and SMC state.";
        
        // Turn off all PLC scenarios
        NLOG(info) << "[ScenarioReset] Turning off PLC scenario 1";
        plcPub.controlScenario(0, 1);
        
        NLOG(info) << "[ScenarioReset] Turning off PLC scenario 2";
        plcPub.controlScenario(0, 2);
        
        NLOG(info) << "[ScenarioReset] Turning off PLC scenario 3";
        plcPub.controlScenario(0, 3);
        
        NLOG(info) << "[ScenarioReset] Turning off PLC scenario 4";
        plcPub.controlScenario(0, 4);
        
        // Reset all state machine states to IDLE
        scenario_1_state_ = SCENARIO_IDLE;
        scenario_2_state_ = SCENARIO_IDLE;
        
        // Reset all completion flags
        scenario_1_1_completed_ = false;
        scenario_1_2_completed_ = false;
        scenario_2_1_completed_ = false;
        scenario_2_2_completed_ = false;
        
        // Reset retry counters
        scenario_1_retry_count_ = 0;
        scenario_2_retry_count_ = 0;
        
        NLOG(info) << "[ScenarioReset] All scenario states reset to IDLE";
    }
    catch (std::exception& e) {
        NLOG(error) << "[EXCEPTION] resetAllScenarios: " << e.what();
    }
}


void StateCheckInitializer::initialize()
{
    ros::NodeHandle nh;
    task_res_pub = nh.advertise<std_msgs::String>("wia_agent/tast_result", 1, true);
    now_task_pub = nh.advertise<std_msgs::String>("wia_agent/now_task", 1, true);
    navifra_cmd_pub = nh.advertise<std_msgs::String>("navifra/cmd", 1, true);
    fork_docking_pub_ = nh.advertise<core_msgs::ForkLift>("/nc_task_manager/fork_docking", 1);
    wingbody_check_pub_ = nh.advertise<std_msgs::Bool>("/wia_agent/wingbody_check", 1);
    live_path_pub = nh.advertise<move_msgs::CoreCommand>("navifra/live_path", 1);
    fork_precontrol_pub_ = nh.advertise<core_msgs::ForkLift>("/nc_task_manager/fork_docking", 10);

    scenario_pub_ = nh.advertise<std_msgs::Int32>("/wia_agent/scenario_control", 10);

    navi_cmd_sub = nh.subscribe("navifra/cmd", 10, &StateCheckInitializer::RecvNaviCmd, this);
    navi_alarm_sub = nh.subscribe("navifra/alarm", 10, &StateCheckInitializer::RecvNaviAlarm, this);
    task_alarm_sub = nh.subscribe("nc_task_manager/task_alarm", 10, &StateCheckInitializer::RecvTaskAlarm, this);
    bms_sub = nh.subscribe("bms_info", 10, &StateCheckInitializer::RecvBmsInfo, this);
    cheonil_info_sub = nh.subscribe("cheonil/read_register", 10, &StateCheckInitializer::RecvCheonilInfo, this);
    slam_trigger_sub = nh.subscribe("slam_trigger", 10, &StateCheckInitializer::RecvSlamTrigger, this);
    pallet_id_sub = nh.subscribe("pallet_id", 10, &StateCheckInitializer::RecvPalletID, this);

    hplc_test_sub = nh.subscribe("/nc_dcs/wingbody_offset", 10, &StateCheckInitializer::RecvHplcTest, this);

    // charge_relay_sub = nh.subscribe("/charge_state", 10, &StateCheckInitializer::RecvCharge, this);
    plc_info_sub = nh.subscribe("plc_info", 10, &StateCheckInitializer::RecvPlcInfo, this);

    scan_data_sub = nh.subscribe("/scan_cloud", 10, &StateCheckInitializer::RecvScan, this);
    obs_data_sub = nh.subscribe("/NaviFra/visualize/ui_obstacle_pos", 10, &StateCheckInitializer::RecvObs, this);

    mission_restart_sub = nh.subscribe("/wia_agent/mission_restart", 10, &StateCheckInitializer::RecvMissionRestart, this);

    wingbody_pos_sub = nh.subscribe("/nc_dcs/wingbody_poses", 10, &StateCheckInitializer::RecvWingBodyPos, this);
    wingbody_approach_move_trigger_sub =
        nh.subscribe("/wia_agent/wingbody_approach_move_trigger", 10, &StateCheckInitializer::RecvWingBodyApproachMoveTrigger, this);
    wingbody_trigger_sub = nh.subscribe("/wia_agent/wingbody_trigger", 10, &StateCheckInitializer::RecvWingBodyTrigger, this);

    scenario_trigger_sub = nh.subscribe("/wia_agent/scenario_complete", 10, &StateCheckInitializer::RecvScenarioTrigger, this);

    status_check_timer = nh.createTimer(ros::Duration(0.2), &StateCheckInitializer::statusCheckCallback, this);
    alarm_check_timer = nh.createTimer(ros::Duration(0.2), &StateCheckInitializer::alarmCheckCallback, this);

    n_start_time_ = Poco::Timestamp();
}

// Fork pre-control helper functions
bool StateCheckInitializer::checkForklift(const std::vector<GoalInfo>& goalList, int current_index, int& out_forklift_index)
{
    // 현재 index부터 연속된 move 체인 찾기
    int i = current_index + 1;

    // 연속된 move들을 건너뛰기
    while (i < goalList.size() && goalList[i].action_type == "move") {
        i++;
    }

    // move 체인  다음이 forklift 또는 lift인지 확인
    if (i < goalList.size()) {
        if (goalList[i].action_type == "forklift" && goalList[i].n_drive_type != 6) {
            out_forklift_index = i;
            return true;
        }
    }

    return false;
}

float StateCheckInitializer::calculateDistance(const std::vector<GoalInfo>& goalList, int current_index, int forklift_index)
{
    auto robotPose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
    if (!robotPose) {
        return 999.0f;  // 거리를 알 수 없으면 큰 값 반환
    }

    // 현재 로봇 위치에서 forklift action 위치까지의 직선 거리 계산
    float f_distance = 0.0f;
    f_distance += std::hypot(
        robotPose->getPosition().x - goalList[current_index].o_pos.GetXm(),
        robotPose->getPosition().y - goalList[current_index].o_pos.GetYm());
    for (int i = current_index + 1; i < forklift_index; i++) {
        f_distance += std::hypot(
            goalList[i].o_pos.GetXm() - goalList[i - 1].o_pos.GetXm(), goalList[i].o_pos.GetYm() - goalList[i - 1].o_pos.GetYm());
    }

    return f_distance;
}

void StateCheckInitializer::publishForkPreControl(int rack_type, int drive_type)
{
    try {
        core_msgs::ForkLift msg;
        msg.n_rack_type = rack_type;
        msg.n_drive_type = drive_type + 100;

        fork_precontrol_pub_.publish(msg);

        NLOG(info) << "[Fork PreControl] Published: " << msg.n_rack_type << ", " << msg.n_drive_type;
    }
    catch (const std::exception& e) {
        NLOG(error) << "[Fork PreControl] Failed to publish: " << e.what();
    }
}

}  // namespace NaviFra
