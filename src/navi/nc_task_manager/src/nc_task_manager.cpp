#include "nc_task_manager/nc_task_manager_pch.h"

#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <boost/format.hpp>
#include <nc_task_manager/nc_bt_engine.h>
#include <nc_task_manager/nc_task_manager.h>
#include <nc_task_manager/plugins/bt_node_types.h>
#include <nc_task_manager/topic/nc_task_manager_topic.h>
#include <std_msgs/Int32MultiArray.h>

#include <mutex>
#include <string>

using TOPIC = NaviFra::NcTaskManagerTopic;
using namespace std;

namespace NaviFra {

NcBehaviorTreeEngine::Ptr g_bt_engine;

TaskManager::TaskManager()
    : activity_(this, &TaskManager::process)
{
    loadBehaviorTree();

    ros::param::param<int>("taskmanager/thread_period", n_thread_period_millisec_, 50);
    // taskmanager -> agent
    rosPub_[TOPIC::TOPIC_TASK_INFO] = nh_.advertise<std_msgs::String>("nc_task_manager/task_info", 10, false);
    rosPub_[TOPIC::TOPIC_TASK_ALARM] = nh_.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 10, false);
    rosPub_[TOPIC::TOPIC_TASK_RESPONSE] = nh_.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_response", 10, false);
    rosPub_[TOPIC::TOPIC_CURRENT_NODE] = nh_.advertise<std_msgs::String>("nc_task_manager/now_node", 10, false);
    rosPub_[TOPIC::TOPIC_TASK_NAME] = nh_.advertise<std_msgs::String>("nc_task_manager/task_name", 10, false);

    // taskmanager -> navigator
    rosPub_[TOPIC::TOPIC_NAV_CMD] = nh_.advertise<std_msgs::String>("navifra/cmd", 10, false);
    rosPub_[TOPIC::TOPIC_LOADED] = nh_.advertise<std_msgs::Bool>("navifra/loaded", 10, false);

    // taskmanager -> hw
    rosPub_[TOPIC::TOPIC_HW_CMD] = nh_.advertise<std_msgs::String>("hwdriver/cmd", 10, false);
    rosPub_[TOPIC::TOPIC_HW_TASK] = nh_.advertise<std_msgs::String>("hwdriver/task", 10, false);

    rosPub_[TOPIC::TOPIC_OUTPUT_COMMAND] = nh_.advertise<std_msgs::String>("output_command", 1, false);
    rosPub_[TOPIC::TOPIC_FORK_INFO] = nh_.advertise<core_msgs::WiaForkInfo>("forkinfo", 1, false);

    // agent
    rosSubs_.push_back(nh_.subscribe("nc_task_manager/task_add", 10, &TaskManager::TaskAddCallback, this));
    rosSubs_.push_back(nh_.subscribe("nc_task_manager/task_cmd", 10, &TaskManager::TaskCmdCallback, this));

    // navigation
    rosSubs_.push_back(nh_.subscribe("navifra/info", 10, &TaskManager::NaviInfoCallback, this));
    rosSubs_.push_back(nh_.subscribe("navifra/alarm", 10, &TaskManager::NaviAlarmCallback, this));

    // hwdriver
    rosSubs_.push_back(nh_.subscribe("hwdriver/info", 10, &TaskManager::DriverInfoCallback, this));
    rosSubs_.push_back(nh_.subscribe("hwdriver/alarm", 10, &TaskManager::DriverAlarmCallback, this));

    rosSubs_.push_back(nh_.subscribe("SeverityMin/task_manager", 1, &TaskManager::RecvSetSeverityMin, this));
    rosSubs_.push_back(nh_.subscribe("SeverityMin", 1, &TaskManager::RecvSetSeverityMin, this));

    // regacy command
    rosSubs_.push_back(nh_.subscribe("map/nodelink", 10, &TaskManager::RecvMapeditorNode, this));
    rosSubs_.push_back(nh_.subscribe("initialpose", 10, &TaskManager::RecvInitialPose, this));

    // undocking start 조건
    rosSubs_.push_back(nh_.subscribe("output_command", 10, &TaskManager::RecvChargeTrigger, this));

    rosSubs_.push_back(nh_.subscribe("charge_state", 10, &TaskManager::RecvChargeState, this));

    rosSubs_.push_back(nh_.subscribe("nc_task_manager/wake_up", 1, &TaskManager::RecvWakeUpWait, this));

    rosSubs_.push_back(nh_.subscribe("/nc_obstacle_detector/object_in_roi", 1, &TaskManager::CheckingCallback, this));
    // loading task 조건
    rosSubs_.push_back(nh_.subscribe("plc/lift_info", 1, &TaskManager::PLCLoadingCallback, this));
    rosSubs_.push_back(nh_.subscribe("plc_info", 1, &TaskManager::PLCInfoCallback, this));
    rosSubs_.push_back(nh_.subscribe("loading_fail", 1, &TaskManager::LoadingFailCallback, this));

    // docking task 조건
    rosSubs_.push_back(nh_.subscribe("docking_success", 10, &TaskManager::DockingSuccessCallback, this));
    // rosSubs_.push_back(nh_.subscribe("charging_success", 10, &TaskManager::ChargingSuccessCallback, this));
    // rosSubs_.push_back(nh_.subscribe("uncharging_success", 10, &TaskManager::UnChargingSuccessCallback, this));
    rosSubs_.push_back(nh_.subscribe("cheonil/read_register", 10, &TaskManager::CheonilRegisterCallback, this));

    rosSubs_.push_back(nh_.subscribe("/wia_agent/wingbody_check", 1, &TaskManager::WingbodyCheckCallback, this));

    commonString_server_ = nh_.advertiseService("nc_task_manager_srv/task_add", &TaskManager::TaskAddSrv, this);

    // lift task 조건 임시
    rosSubs_.push_back(nh_.subscribe("fork_lift_reached", 1, &TaskManager::ForkLiftCallback, this));

    start();
}

TaskManager::~TaskManager()
{
    stop();
    commonString_server_.shutdown();
    g_bt_engine.reset();

    for (auto pub : rosPub_) {
        pub.second.shutdown();
    }
}

void TaskManager::RecvSetSeverityMin(const std_msgs::Int16 msg)
{
    if (msg.data < 0 && msg.data > 5)
        return;
    NaviFra::Logger::get().SetSeverityMin((severity_level)msg.data);
}

bool TaskManager::TaskAddSrv(core_msgs::CommonStringRequest& req, core_msgs::CommonStringResponse& res)
{
    try {
        Poco::JSON::Parser parser;
        NLOG(info) << "req.data " << req.data;
        Poco::Dynamic::Var result = parser.parse(req.data);
        Poco::JSON::Object::Ptr obj = result.extract<Poco::JSON::Object::Ptr>();
        std::string action = obj->get("action").extract<std::string>();
        std::string uuid = obj->get("uuid").extract<std::string>();
        Poco::JSON::Array::Ptr arrary_data = obj->getArray("data");

        if (arrary_data->size() == 0 || currentMode_ == Mode::MANUAL) {
            //로봇 수동모드일때 task 안받음
            NLOG(error) << "robot mode is manual" << int(currentMode_);
            res.success = false;
            return true;
        }

        auto shared_queue =
            blackboard_->getAny("tasks")->empty() ? std::make_shared<std::deque<Task>>() : blackboard_->get<BT::SharedQueue<Task>>("tasks");

        bool b_check = false;
        if (blackboard_->getAny("tasks")->empty()) {
            b_check = true;
        }
        NLOG(info) << "Array size: " << arrary_data->size() << ", " << b_check;  // 배열 크기 출력

        // for (size_t index = 0; index < arrary_data->size(); index++) {
        //     try {
        //         // 직접적으로 get() 메소드 사용
        //         Poco::Dynamic::Var item = arrary_data->get(index);

        //         // 항목을 문자열로 변환하여 출력
        //         std::string item_str = item.toString(); // Var를 문자열로 변환
        //         NLOG(info) << "Element at index " << index << ": " << item_str; // 변환된 문자열 출력
        //     } catch (const std::exception& e) {
        //         NLOG(error) << "Error processing array element at index " << index << ": " << e.what();
        //     }
        // }

        for (size_t index = 0; index < arrary_data->size(); index++) {
            auto tmp_object = arrary_data->getObject(index);
            NLOG(info) << "Retrieved object at index " << index;

            // 객체의 정보 로깅 (객체의 구조에 따라 필요한 키 값을 로그로 출력)
            // 예: task_id나 task_name 등이 있다고 가정
            if (tmp_object->has("type")) {
                NLOG(info) << "type ID: " << tmp_object->getValue<std::string>("type");
            }
            if (tmp_object->has("uuid")) {
                NLOG(info) << "uuid ID: " << tmp_object->getValue<std::string>("uuid");
            }
            else {
                NLOG(info) << "else";
            }
            // if (tmp_object->has("task_name")) {
            //     NLOG(info) << "Task Name: " << tmp_object->getValue<std::string>("task_name");
            // }
            shared_queue->push_back(Task(arrary_data->getObject(index)));

            if (shared_queue->back().type() == TYPE.ACTION)  // action add, nate
            {
                NLOG(info) << "Action Task: " << shared_queue->back().uuid_;

                std::string s_action_uuid = shared_queue->back().uuid_;  // task uuid 만 저장
                shared_queue->pop_back();  // 기존것 날리기(그냥 트리거이므로)

                NaviFra::NaviNode o_node = o_waypoint_planner_.GetNode(s_current_node_id_);  // 노드의 action 가져오기
                std::string s_action_name = o_node.GetActionName();  // 추상화 action 이름 (ui에서 설정한)
                std::vector<string> vec_subaction;
                try {
                    NLOG(info) << "s_action_name: " << s_action_name;
                    ros::param::get("action/" + s_action_name, vec_subaction);
                }
                catch (const std::exception& e) {
                    NLOG(error) << e.what() << '\n';
                }

                NLOG(info) << "vec_subaction size: " << vec_subaction.size();
                if (vec_subaction.size() == 0) {
                    NLOG(error) << "No subaction found for action: " << s_action_name;
                    continue;  // action이 없으면 그냥 넘어감
                }
                for (const auto& subaction : vec_subaction) {
                    NLOG(info) << "Subaction: " << subaction;
                    Task task;
                    task.uuid_ = s_action_uuid;  // action task uuid
                    task.type_ = subaction;  // subaction 이름
                    shared_queue->emplace_back(task);  // subaction task 추가
                }
            }
        }

        if (shared_queue->back().arrvieAlign()) {
            shared_queue->front().b_arrive_align_ = true;
            NLOG(info) << "shared_queue.at(0) arrive " << shared_queue->at(0).b_arrive_align_;
            NLOG(info) << "shared_queue(0) uuid" << shared_queue->at(0).uuid_;
        }

        NLOG(info) << "shared queue push back finished";

        bool b_move = true;
        for (int i = 0; i < shared_queue->size(); i++) {
            NLOG(info) << "shared_queue->at(i).type() " << shared_queue->at(i).type();
            if (shared_queue->at(i).type() != TYPE.MOVE) {
                b_move = false;
                break;
            }
        }
        NLOG(info) << "b_move " << b_move;

        if (b_move)  // 이동 할 때
        {
            string s_status;
            blackboard_->get("s_robot_status", s_status);
            std::chrono::duration<double> sec = std::chrono::system_clock::now() - tp_move_;
            std::chrono::duration<double> sec2 = std::chrono::system_clock::now() - tp_idle_;

            NLOG(info) << "sec.count() " << sec.count() << " / "
                       << "s_status " << s_status;
            NLOG(info) << "sec2.count() " << sec2.count() << " / "
                       << "s_status " << s_status;

            if (s_status == "idle" && sec.count() > 1 && sec2.count() > 10) {  // 이상상황 시 클리어
                NLOG(info) << "clear!!!!!!";
                shared_queue->clear();
                for (size_t index = 0; index < arrary_data->size(); index++) {
                    shared_queue->push_back(Task(arrary_data->getObject(index)));
                }
            }
            if (MoveProcess(shared_queue)) {
                tp_move_ = std::chrono::system_clock::now();
                NLOG(info) << "need_update_goal";

                blackboard_->set("tasks", shared_queue);
                blackboard_->set("current_task", shared_queue->at(0));
                blackboard_->set("need_update_goal", true);
            }
            else {
                res.success = false;
                NLOG(info) << "pop back / path create fail";
                shared_queue->pop_back();
                blackboard_->set("tasks", shared_queue);

                static ros::Publisher error_pub = nh_.advertise<std_msgs::Int64>("navifra/error", 10, false);
                std_msgs::Int64 error_msg;
                error_msg.data = core_msgs::NaviAlarm::ERROR_PATH_NOT_CREATED;  // path create fail
                error_pub.publish(error_msg);
                return false;
            }
            NLOG(info) << "move not loading ";
            // NLOG("task alarm start");
        }
        else {
            blackboard_->set("tasks", shared_queue);
            blackboard_->set("current_task", shared_queue->at(0));
            Task test;
            blackboard_->get("current_task", test);
            NLOG(info) << test.type();
        }

        NLOG(info) << "g_bt_engine start";
        g_bt_engine->start();
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
        res.success = false;
        return false;
    }

    res.success = true;
    return true;
}

bool TaskManager::MoveProcess(BT::SharedQueue<Task>& tasks)
{
    try {
        std::vector<float> robot_outline;
        ros::param::get("obstacle/outline", robot_outline);
        float f_robot_size = 1;
        if (robot_outline.size() == 4) {
            LOG_INFO("Read Robot Outline successfully.");
            f_robot_size = fabs(robot_outline.at(3)) + fabs(robot_outline.at(2));
        }
        NaviFra::Pos o_last_pos = o_robot_pos_;
        NLOG(info) << "o_last_pos " << o_last_pos.GetXm() << " / " << o_last_pos.GetYm();
        NaviFra::Pos::DriveInfo_t o_drive;
        for (auto& task : *tasks) {
            if (task.type() != TYPE.MOVE)
                break;
            NLOG(info) << "task.type() == TYPE.MOVE "
                       << " task.vec_move_data_.size() " << task.vec_move_data_.size();
            std::vector<Task::MoveData> vec_gen_data;

            if ((task.targetNode().size() > 0 || task.path_nodes_.size() > 0) && b_map_loaded_) {
                if (task.vec_move_data_.size() > 0) {  // 경로를 이미 한번 생성한 경우
                    o_last_pos.SetXm(task.vec_move_data_.back().f_x_m);
                    o_last_pos.SetYm(task.vec_move_data_.back().f_y_m);
                    NLOG(info) << "경로를 이미 한번 생성한 경우 o_last_pos " << o_last_pos.GetXm() << " / " << o_last_pos.GetYm();
                    continue;
                }
                std::vector<NaviFra::NaviNode> vec_path;
                NLOG(info) << "task.targetNode().size() " << task.targetNode().size();
                NLOG(info) << "task.path_nodes_.size() " << task.path_nodes_.size();
                if (task.targetNode().size() > 0 || task.path_nodes_.size() == 1)  // goal_id만 받았을 경우
                {
                    NaviFra::NaviNode o_node_target;
                    if (task.targetNode().size() > 0)
                        o_node_target = o_waypoint_planner_.GetNode(task.targetNode());
                    else if (task.path_nodes_.size() == 1) {
                        o_node_target = o_waypoint_planner_.GetNode(task.path_nodes_[0]);
                        o_node_target.SetDeg(task.finish_angle_);
                        o_node_target.SetType(NaviFra::Pos::NODE_TYPE::SPIN_TURN);
                        NLOG(info) << "task.finish_angle_ " << task.finish_angle_;
                    }
                    float f_dist = hypot(o_last_pos.GetXm() - o_node_target.GetXm(), o_last_pos.GetYm() - o_node_target.GetYm());
                    NLOG(info) << "f_dist " << f_dist << " o_node_target " << o_node_target.GetXm() << " " << o_node_target.GetYm();
                    if (f_dist > 0.07)  // 일정거리 이상 떨어져 있을 경우
                        vec_path = o_waypoint_planner_.PlanPath(o_last_pos, task.targetNode());
                    else {  // 제자리 턴에 대한 경우
                        if (o_node_target.GetType() == NaviFra::Pos::NODE_TYPE::SPIN_TURN)
                            vec_path.emplace_back(o_node_target);
                        else {
                            NLOG(error) << "Spin turn not created";
                            return false;
                        }
                    }
                }
                else if (task.path_nodes_.size() > 1)  // node list를 받았을 경우
                {
                    // NLOG(info) << "Please...";
                    vec_path = o_waypoint_planner_.PlanPath(task.path_nodes_);
                }
                NLOG(info) << "vec_path size : " << vec_path.size();
                if (vec_path.size() == 0) {
                    NLOG(error) << "Path not created";
                    return false;
                }
                for (int i = 0; i < vec_path.size(); i++) {
                    Task::MoveData move_data;
                    move_data.s_id = vec_path.at(i).GetID();
                    move_data.s_name = vec_path.at(i).GetName();
                    if (i != vec_path.size() - 1) {
                        float f_dist =
                            hypot(vec_path.at(i + 1).GetXm() - vec_path.at(i).GetXm(), vec_path.at(i + 1).GetYm() - vec_path.at(i).GetYm());
                        if (f_dist < 0.01) {
                            NLOG(info) << "f_dist < 0.01 / curve "
                                       << o_waypoint_planner_.GetDriveInfo(vec_path.at(i), vec_path.at(i + 1)).e_curve_type;
                            continue;
                        }
                        o_drive = o_waypoint_planner_.GetDriveInfo(vec_path.at(i), vec_path.at(i + 1));
                    }
                    NLOG(info) << "i " << i << " n_drive_type " << o_drive.e_drive_type << " move_data.s_name " << vec_path.at(i).GetName();

                    move_data.n_drive_type = o_drive.e_drive_type;
                    move_data.f_speed_ms = o_drive.f_linear;
                    move_data.f_x_m = vec_path.at(i).GetXm();
                    move_data.f_y_m = vec_path.at(i).GetYm();
                    move_data.f_angle_deg = vec_path.at(i).GetDeg();
                    NLOG(info) << "node type " << vec_path.at(i).GetType();
                    if (i == vec_path.size() - 1 && vec_path.at(i).GetType() == NaviFra::Pos::NODE_TYPE::SPIN_TURN) {
                        move_data.b_arrive_align = true;
                    }

                    if (o_drive.b_avoidance_left && o_drive.b_avoidance_right) {
                        move_data.n_avoid_type = 1;
                    }
                    else if (o_drive.b_avoidance_left) {
                        move_data.n_avoid_type = 2;
                    }
                    else if (o_drive.b_avoidance_right) {
                        move_data.n_avoid_type = 3;
                    }
                    if (move_data.n_avoid_type > 0) {
                        move_data.f_avoid_lanewidth = o_drive.n_avoidance_step * f_robot_size;
                    }

                    move_data.list_f_obstacle_margin.at(0) = o_drive.f_obstacle_outline_margin_front;
                    move_data.list_f_obstacle_margin.at(1) = o_drive.f_obstacle_outline_margin_rear;
                    move_data.list_f_obstacle_margin.at(2) = o_drive.f_obstacle_outline_margin_left;
                    move_data.list_f_obstacle_margin.at(3) = o_drive.f_obstacle_outline_margin_right;

                    move_data.b_start_quick = !(o_drive.b_start_smooth);
                    move_data.b_stop_quick = !(o_drive.b_stop_smooth);

                    NLOG(info) << "move_data.s_name : " << move_data.s_name;

                    vec_gen_data.push_back(move_data);
                    NLOG(info) << "o_drive.e_curve_type " << o_drive.e_curve_type;
                    // 곡선을 위한 가상의 점 추가
                    if (o_drive.e_curve_type == NaviFra::Pos::LINE_TYPE::CURVE && i != vec_path.size() - 1) {
                        Task::MoveData move_data_curve = move_data;

                        CaclPosDenominator(
                            move_data_curve, vec_path.at(i), vec_path.at(i + 1), o_drive.f_circle_pos_x, o_drive.f_circle_pos_y);
                        move_data_curve.s_name = "V" + move_data_curve.s_name;
                        move_data_curve.f_angle_deg = vec_path.at(i + 1).GetDeg();
                        move_data_curve.f_curve_radius =
                            hypot(vec_path.at(i).GetXm() - move_data_curve.f_x_m, vec_path.at(i).GetYm() - move_data_curve.f_y_m);
                        vec_gen_data.push_back(move_data_curve);
                    }
                }
                task.end_node_ = vec_path.back().GetID();
                task.vec_move_data_ = vec_gen_data;
            }
            else if (task.vec_move_data_.size() > 0) {
                NLOG(info) << "already task.vec_move_data_.size() > 0";
            }
        }
    }
    catch (std::exception& ex) {
        NLOG(error) << ex.what();
    }
    return true;
}

void TaskManager::CaclPosDenominator(
    Task::MoveData& move_data_curve, const NaviFra::Pos& o_pos1, const NaviFra::Pos& o_pos2, const float& cx, const float& cy)
{
    // 2. 접선 방정식의 계수 계산
    float a1 = o_pos1.GetXm() - cx;
    float b1 = o_pos1.GetYm() - cy;
    float c1 = a1 * o_pos1.GetXm() + b1 * o_pos1.GetYm();

    float a2 = o_pos2.GetXm() - cx;
    float b2 = o_pos2.GetYm() - cy;
    float c2 = a2 * o_pos2.GetXm() + b2 * o_pos2.GetYm();

    // 3. 교점 계산
    float denominator = a1 * b2 - a2 * b1;
    move_data_curve.f_x_m = (c1 * b2 - c2 * b1) / denominator;
    move_data_curve.f_y_m = (a1 * c2 - a2 * c1) / denominator;
}

void TaskManager::TaskAddCallback(const core_msgs::AddTask::ConstPtr& msg)
{
    // Not use
    Task task;
}

void TaskManager::pause()
{
    Task task;
    if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
        std_msgs::String msg;
        msg.data = CMD.PAUSE;
        publish(NcTaskManagerTopic::TOPIC_NAV_CMD, msg);
        publish(NcTaskManagerTopic::TOPIC_HW_CMD, msg);
    }

    NLOG(info) << "Task Pasue";
}  // namespace NaviFra

void TaskManager::resume()
{
    Task task;
    if (blackboard_->getAny("tasks")->empty()) {
        NLOG(error) << "task is empty";
        return;
    }
    BT::SharedQueue<Task> tasks;
    blackboard_->get("tasks", tasks);
    if (tasks->size() > 0) {
        if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
            std_msgs::String msg;
            msg.data = CMD.RESUME;
            publish(NcTaskManagerTopic::TOPIC_NAV_CMD, msg);
            publish(NcTaskManagerTopic::TOPIC_HW_CMD, msg);
        }
        NLOG(info) << "Task Resume";
    }
    else {
        NLOG(info) << "Task size is less than 0";
    }
}

void TaskManager::NavifraCmdPub(const std::string& cmd)
{
    std_msgs::String msg;
    msg.data = cmd;
    publish(TOPIC::TOPIC_NAV_CMD, msg);
}

void TaskManager::DriverCmdPub(const std::string& cmd)
{
    std_msgs::String msg;
    msg.data = cmd;
    publish(TOPIC::TOPIC_HW_CMD, msg);
}

void TaskManager::TaskCmdCallback(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("Task CmdCallback %s", msg->data.c_str());
    if (msg->data == CMD.CANCEL) {
        TaskCancel();
        publish(TOPIC::TOPIC_NAV_CMD, *msg);
        core_msgs::WiaForkInfo fork_info;
        fork_info.b_cancel = true;
        publish(TOPIC::TOPIC_FORK_INFO, fork_info);
    }
    else if (msg->data == CMD.PAUSE) {
        pause();
    }
    else if (msg->data == CMD.RESUME) {
        resume();
    }
}

void TaskManager::NaviInfoCallback(const core_msgs::NavicoreStatus::ConstPtr& msg)
{
    o_robot_pos_.SetXm(msg->f_robot_pos_x_m);
    o_robot_pos_.SetYm(msg->f_robot_pos_y_m);
    o_robot_pos_.SetDeg(msg->f_robot_pos_deg);
    std::string s_current_node_id = msg->s_current_node_id;
    blackboard_->set("current_node", msg->s_current_node_id);
    blackboard_->set("o_robot_pos", o_robot_pos_);
    blackboard_->set("s_robot_status", msg->s_status);

    if (msg->s_status != "idle") {
        tp_idle_ = std::chrono::system_clock::now();
    }

    // 로봇 정지 상태에서 현재 노드 찾기

    std::vector<NaviFra::NaviNode> o_now_nodes = o_waypoint_planner_.FindNeighboringNodeVec(o_robot_pos_, 5);
    if (msg->s_status == "idle") {
        bool b_find = false;
        for (int i = 0; i < o_now_nodes.size(); i++) {
            float f_node_robot_dist = CoreCalculator::CalcPosDistance_(o_now_nodes.at(i), o_robot_pos_);
            if (msg->s_current_node_id == o_now_nodes.at(i).GetID() && f_node_robot_dist < 0.5) {
                b_find = true;
                break;
            }
        }
        if (b_find == false) {
            bool b_idle_find = false;
            for (int i = 0; i < o_now_nodes.size(); i++) {
                // NLOG(info) << "o_now_nodes " << o_now_nodes.at(i).GetName();
                if (o_now_nodes.at(i).GetName().substr(0, 3) != "ARC" && o_now_nodes.at(i).GetName().substr(0, 2) != "QR") {
                    float f_node_robot_dist = CoreCalculator::CalcPosDistance_(o_now_nodes.at(i), o_robot_pos_);
                    if (f_node_robot_dist < 0.5) {
                        // 현재 노드 업데이트
                        std_msgs::String msg;
                        msg.data = o_now_nodes.at(i).GetName() + "|" + o_now_nodes.at(i).GetID();
                        rosPub_[TOPIC::TOPIC_CURRENT_NODE].publish(msg);
                        b_idle_find = true;
                        s_current_node_id = o_now_nodes.at(i).GetID();
                        break;
                    }
                }
            }
            if (b_idle_find == false) {
                std_msgs::String msg;
                msg.data = "0|0";
                rosPub_[TOPIC::TOPIC_CURRENT_NODE].publish(msg);
            }
        }
    }
    s_current_node_id_ = s_current_node_id;

    Task current_task, previous_task;
    std::string s_current_task = "";
    std::string s_previous_task = "";
    if (!blackboard_->getAny("previous_task")->empty() && blackboard_->get("previous_task", previous_task)) {
        s_previous_task = previous_task.uuid_;
    }
    if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", current_task)) {
        s_current_task = current_task.uuid_;

        std_msgs::String task_name;
        std::string type = current_task.type_;
        if (type != TYPE.MOVE && type.size() != 0) {
            task_name.data = type;
        }
        publish(NcTaskManagerTopic::TOPIC_TASK_NAME, task_name);
    }
    std::string s_msg = s_current_task + "/" + s_previous_task;
    TaskPubStatus(s_msg);
}

void TaskManager::NaviAlarmCallback(const core_msgs::NaviAlarm::ConstPtr& msg)
{
    // LOG_INFO("Task NaviAlarmCallback %d", msg->alarm);
    if (msg->alarm == core_msgs::NaviAlarm::GOAL_ARRIVED) {
        NLOG(info) << "GOAL_ARRIVED";

        Task task;
        if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
            if (task.type() == TYPE.MOVE) {
                blackboard_->set("goal_reached", true);

                float f_dist =
                    hypot(task.vec_move_data_.back().f_x_m - o_robot_pos_.GetXm(), task.vec_move_data_.back().f_y_m - o_robot_pos_.GetYm());
                NLOG(info) << "f_dist max " << f_dist;
                // if (f_dist < 0.3) {
                // NLOG(info) << "GOAL_ARRIVED! " << f_dist;
                // blackboard_->set("goal_reached", true);
                // }
                // core_msgs::TaskAlarm alarm;
                // alarm.alarm = ALARM.DONE;
                // alarm.uuid = task.uuid();
                // alarm.type = task.type();
                // publish(TOPIC::TOPIC_TASK_ALARM, alarm);
            }
            else if (task.type() == TYPE.UNDOCKING) {
                blackboard_->set("undocking_reached", "true");

                // if (f_dist < 0.3) {
                // NLOG(info) << "GOAL_ARRIVED! " << f_dist;
                // blackboard_->set("goal_reached", true);
                // }
                // core_msgs::TaskAlarm alarm;
                // alarm.alarm = ALARM.DONE;
                // alarm.uuid = task.uuid();
                // alarm.type = task.type();
                // publish(TOPIC::TOPIC_TASK_ALARM, alarm);
            }
        }
    }
    else {
        // LOG 남겨야 됨
    }
}

void TaskManager::DriverInfoCallback(const std_msgs::String::ConstPtr& msg)
{
    /*
        HW Command 를 사용 하는 애들이 여러개 인데 진행 시작 여부를 이것의
       Status로 알아야 할 듯 하다. Behavior Tree의 Blackboard에 상태 값을 저장
       하는 무언 가를 만들어 그것을 읽어서 이용 하도록 수정 이 필요 해 보인다.
    */
}

void TaskManager::DriverAlarmCallback(const std_msgs::String::ConstPtr& msg)
{
    NLOG(trace) << "driverAlarmCallback " << msg->data;
    if (msg->data == "done") {
        NLOG(info) << "driverAlarmCallback " << msg->data;
        blackboard_->set("driverDoneReceived", true);
    }
}

void TaskManager::TaskPubStatus(const string& data)
{
    std_msgs::String status;
    status.data = data;
    publish(NcTaskManagerTopic::TOPIC_TASK_INFO, status);
}

void TaskManager::TaskPubAlarm(const string& uuid, const string& data)
{
    NLOG(info) << boost::format("Task PubAlarm %1%, uuid : %2%") % data.c_str() % uuid.c_str();

    core_msgs::TaskAlarm alarm;
    alarm.uuid = uuid;
    alarm.alarm = data;

    publish(NcTaskManagerTopic::TOPIC_TASK_ALARM, alarm);
}

void TaskManager::RecvMapeditorNode(const core_msgs::JsonList& msg)
{
    LOG_INFO("RecvMapeditorNode");
    core_msgs::JsonList jsonlist = msg;
    o_waypoint_planner_.SetMap(jsonlist);
    b_map_loaded_ = true;
}

void TaskManager::RecvInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    // LOG_INFO("RecvInitialPose All Clear");

    // if (g_bt_engine->isRunning()) {
    //     NLOG(info) << "requrest clear Command";
    //     g_bt_engine->cancel();
    //     auto shared_queue = std::make_shared<std::deque<Task>>();
    //     blackboard_->set("tasks", shared_queue);
    // }
}

void TaskManager::RecvChargeTrigger(const std_msgs::String::ConstPtr& msg)
{
    Task task;
    if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
        std::string s_recv_msg = msg->data;
        if (task.type() == TYPE.UNDOCKING && s_recv_msg == "charge_stop_done") {
            blackboard_->set("uncharge_received", true);
            NLOG(info) << s_recv_msg << " received. undocking is available";
        }
    }
}

void TaskManager::RecvChargeState(const std_msgs::Bool::ConstPtr& msg)
{
    Task task;
    if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
        bool b_recv_charge_state_msg = msg->data;
        if (task.type() == TYPE.UNDOCKING && !b_recv_charge_state_msg) {
            blackboard_->set("uncharge_received", true);
            NLOG(info) << "b_recv_charge_state_msg : " << b_recv_charge_state_msg;
        }

        NLOG(info) << "b_recv_charge_state_msg : " << b_recv_charge_state_msg;

        if (task.type() == TYPE.DOCKING && b_recv_charge_state_msg) {
            NLOG(info) << "task.docking_type() : " << task.docking_type() << " docking_charge_result_ : " << docking_charge_result_;
            if (task.docking_type() == "aruco charge") {
                if (docking_charge_result_ == true) {
                    blackboard_->set("docking_reached", "true");

                    NLOG(info) << "docking_charge_reached Success !! ";
                }
                else if (docking_charge_result_ == false) {
                    NLOG(info) << "docking_charge_reached Error !! ";
                    TaskCancel();
                }
                NLOG(info) << "aruco_charge : " << task.docking_type();
            }
        }
    }
}

void TaskManager::PLCLoadingCallback(const task_msgs::LiftInfo::ConstPtr& msg)
{
    NLOG(trace) << "LoadingCallback ";
    std_msgs::Bool b_load;
    b_load.data = false;
    if (msg->lift_top) {
        b_load.data = true;
    }
    publish(NcTaskManagerTopic::TOPIC_LOADED, b_load);

    Task task;
    if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
        if (task.type() == TYPE.LOADING) {
            if (msg->lift_top) {
                NLOG(info) << "loading success";
                blackboard_->set("loading_received", "true");
            }
        }
        else if (task.type() == TYPE.UNLOADING) {
            if (msg->lift_bottom) {
                NLOG(info) << "unloading success";
                blackboard_->set("unloading_received", "true");
            }
        }
    }
}

void TaskManager::PLCInfoCallback(const core_msgs::PLCInfo::ConstPtr& msg)
{
    Task task;
    if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
        auto tmp_data = *msg;
        if (tmp_data.front_up_px && tmp_data.rear_up_px) {
            blackboard_->set("lift_up", "true");
        }
        else if (tmp_data.front_down_px && tmp_data.rear_down_px) {
            blackboard_->set("lift_down", "true");
        }
    }
}

void TaskManager::LoadingFailCallback(const std_msgs::Bool::ConstPtr& msg)
{
    auto loading_result = msg->data;
    NLOG(trace) << "loading_result : " << loading_result;

    Task task;
    if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
        if (task.type() == TYPE.LOADING || task.type() == TYPE.UNLOADING) {
            if (loading_result == true) {
                blackboard_->set("loading_received", "error");
                blackboard_->set("unloding_received", "error");
            }
        }
    }
    else {
        NLOG(error) << "TaskManager::LoadingFailCallback current_task is empty";
    }
}

void TaskManager::DockingSuccessCallback(const std_msgs::Bool::ConstPtr& msg)
{
    auto docking_result = msg->data;
    docking_charge_result_ = msg->data;
    NLOG(trace) << "docking_success_result : " << docking_result;

    Task task;
    if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
        // NLOG(info) << task.type();
        if (task.type() == TYPE.DOCKING || task.type() == TYPE.UNDOCKING) {
            if (task.docking_type() == "aruco") {
                if (docking_result == true) {
                    blackboard_->set("docking_reached", "true");
                }
                else if (docking_result == false) {
                    NLOG(info) << "docking_reached Error !! ";
                    TaskCancel();
                }
            }
            else {
                if (docking_result == true) {
                    Poco::Thread::sleep(7000);
                    std_msgs::String msg;
                    msg.data = "charge_start";
                    rosPub_[TOPIC::TOPIC_OUTPUT_COMMAND].publish(msg);

                    NLOG(info) << "Docking Charge Start !! ";
                }
                else if (docking_result == false) {
                    NLOG(info) << "Docking Charge Not start !! ";
                }
            }
        }
    }
    else {
        NLOG(error) << "TaskManager::DockingSuccessCallback current_task is empty";
    }
}

void TaskManager::CheonilRegisterCallback(const core_msgs::CheonilReadRegister::ConstPtr& msg)
{
    if (!blackboard_->getAny("current_task")->empty()) {
        Task current_task;
        std::string s_charge_start_received;

        bool b_charge_stop_received, b_charge_stop_trigger, b_loaded;
        int n_cheonil_cnt = msg->command_num;

        blackboard_->get("charge_stop_trigger", b_charge_stop_trigger);

        if (n_cheonil_cnt % 10 == 1 && !b_charge_stop_trigger) {
            NLOG(info) << "Charge Stop Trigger -> false";
            blackboard_->set<bool>("charge_stop_trigger", true);
        }
        blackboard_->get("charge_start_received", s_charge_start_received);
        blackboard_->get<bool>("charge_stop_received", b_charge_stop_received);
        blackboard_->get("current_task", current_task);

        int n_charge_state = static_cast<int>(msg->charge_state);

        if (current_task.type() == TYPE.CHARGING && n_charge_state == 1 && s_charge_start_received == "false") {
            // 충전 시작 완료
            blackboard_->set("charge_start_received", "true");
        }

        else if (n_charge_state == 0 && !b_charge_stop_received) {
            //충전 종료 완료
            blackboard_->set<bool>("charge_stop_received", true);
        }

        b_loaded = msg->load_state;
        blackboard_->set<bool>("load_state_received", b_loaded);
    }
}

void TaskManager::RecvWakeUpWait(const core_msgs::HacsWakeup::ConstPtr& msg)
{
    bool b_wake_up = msg->wake_up;
    std::string wait_id = msg->id;
    if (b_wake_up) {
        blackboard_->set("wait_id", wait_id);
    }
    else {
        blackboard_->set("wait_id", "");
    }

    NLOG(info) << " Action wait_id : " << wait_id;
}

void TaskManager::CheckingCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    std::stringstream ss;
    NLOG(trace) << "CheckingCallback data: " << ss.str();
    for (auto value : msg->data) {
        ss << value << " ";
        NLOG(info) << value << " ";
    }

    blackboard_->set("object_in_roi_data", msg->data);

    std::vector<int> object_in_roi_data;
    if (blackboard_->get("object_in_roi_data", object_in_roi_data)) {
        std::stringstream retrieved_ss;
        for (auto value : object_in_roi_data) {
            retrieved_ss << value << " ";
        }
        NLOG(info) << "Blackboard stored data: " << retrieved_ss.str();
    }
    else {
        NLOG(error) << "Failed to retrieve object_in_roi_data from blackboard";
    }

    Task task;
    if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", task)) {
        if (!msg->data.empty()) {
            bool hasZero = false;
            // 배열의 모든 요소를 검사하여 하나라도 0이면 hasZero를 true로 설정
            if (msg->data.size() == 1) {
                hasZero = true;  // 크기가 1이면 무조건 true
            }
            else {
                for (const auto& value : msg->data) {
                    if (value == 0) {
                        hasZero = true;
                        break;
                    }
                }
            }
            if (hasZero) {
                NLOG(info) << "Task type: " << task.type() << ", at least one element is 0";
                if ((task.type() == TYPE.PICKUP_CHECKING) || (task.type() == TYPE.RETURN_CHECKING || task.type() == TYPE.TO_CHECKING)) {
                    blackboard_->set("checking_start_received", "true");
                    NLOG(info) << "TYPE.CHECKING: checking_start_received set to true";
                }
            }
            else {
                NLOG(info) << "0 없음";
                if (task.type() == TYPE.PICKUP_CHECKING || task.type() == TYPE.RETURN_CHECKING) {
                    NLOG(info) << "TO_CHECKING";
                    blackboard_->set("checking_start_received", "true");
                }
                else {
                    NLOG(info) << "Task type: " << task.type() << ", no zero found in obstacle data";
                }
            }
        }
    }
}

void TaskManager::ForkLiftCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        blackboard_->set("fork_lift_reached", true);

        NLOG(info) << "Fork Lift Reached Call Back ";
    }
}

void TaskManager::WingbodyCheckCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data){
        blackboard_->set("wingbody_perception_reached", true);

        NLOG(info) << "Wingbody Perception Reached Call Back";
    }
}

bool TaskManager::loadBehaviorTree(const std::string& bt_xml_filename)
{
    // clang-format off
    static const char* xml_tree = R"(
        <root BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <RetryUntilSuccessful num_attempts="5">
                    <Fallback>
                        <ReactiveSequence name="Move">
                                <IsStopChargeCondition current_task="{current_task}" charge_stop_received="{charge_stop_received}" charge_stop_trigger = "{charge_stop_trigger}"/>
                                <IsMove tasks="{tasks}" current_task="{current_task}" need_update_goal="{need_update_goal}" goals="{goals}" s_robot_status="{s_robot_status}" o_robot_pos="{o_robot_pos}" charge_stop_received="{charge_stop_received}" load_state_received="{load_state_received}"/>
                                <MoveToGoal goals="{goals}" updated_goals="{updated_goals}" current_task="{current_task}" goal_reached="{goal_reached}"/>
                                <GoalReached goals="{goals}" current_task="{current_task}" current_node="{current_node}" previous_task="{previous_task}" tasks="{tasks}" o_robot_pos="{o_robot_pos}" goal_reached="{goal_reached}"/>
                        </ReactiveSequence>
                        <Sequence name="Manual">
                            <IsManual current_task="{current_task}"/>
                            <WaitForManual current_task="{current_task}" tasks="{tasks}" driverDoneReceived="{driverDoneReceived}"/>
                        </Sequence>
                        <Sequence  name="Turn">
                            <IsTurn current_task="{current_task}" current_node="{current_node}" />
                            <TurnReached current_task="{current_task}" tasks="{tasks}" turn_reached="{turn_reached}"/>
                        </Sequence>
                        <Sequence  name="Action">
                            <IsAction current_task="{current_task}"/>
                            <ActionReached current_task="{current_task}" tasks="{tasks}" action_reached="{action_reached}" />
                        </Sequence>
                        <Sequence  name="Charging">
                            <IsCharging current_task="{current_task}"/>
                            <ChargingSuccess current_task="{current_task}" tasks="{tasks}" charge_start_received="{charge_start_received}" />
                        </Sequence>
                        <Sequence  name="Docking">
                            <IsDocking current_task="{current_task}" current_node="{current_node}" s_robot_status="{s_robot_status}"/>
                            <DockingReached current_task="{current_task}" tasks="{tasks}" docking_reached="{docking_reached}"/>
                        </Sequence>
                        <Sequence  name="Wait">
                            <IsStopChargeCondition current_task="{current_task}" charge_stop_received="{charge_stop_received}" charge_stop_trigger = "{charge_stop_trigger}"/>
                            <IsWaitCondition current_task="{current_task}"/>
                            <WaitSuccessCondition current_task="{current_task}" tasks="{tasks}" wait_success="{wait_success}" wait_id="{wait_id}" wait_cancel="{wait_cancel}" />
                        </Sequence>
                        <Sequence  name="UnDocking">
                            <IsUnDocking current_task="{current_task}" need_update_goal="{need_update_goal}" goals="{goals}" s_robot_status="{s_robot_status}" o_robot_pos="{o_robot_pos}" uncharge_received="{uncharge_received}"/>
                            <UnDockingReached current_task="{current_task}" tasks="{tasks}" undocking_reached="{undocking_reached}"/>
                        </Sequence>
                          <Sequence  name="Checking">
                            <IsChecking current_task="{current_task}"/>
                            <CheckingSuccess current_task="{current_task}" tasks="{tasks}" checking_start_received="{checking_start_received}" />
                        </Sequence>
                        <Sequence  name="Loading">
                            <IsLoading current_task="{current_task}"/>
                            <LoadingSuccess current_task="{current_task}" tasks="{tasks}" loading_received="{loading_received}"unloading_received="{unloading_received}" lift_up="{lift_up}" lift_down="{lift_down}"/>
                        </Sequence>
                        <Sequence  name="ForkLift">
                            <IsStopChargeCondition current_task="{current_task}" charge_stop_received="{charge_stop_received}" charge_stop_trigger = "{charge_stop_trigger}"/>
                            <IsForkLift current_task="{current_task}"/>
                            <ForkLiftReached current_task="{current_task}" tasks="{tasks}" fork_lift_reached="{fork_lift_reached}" wingbody_perception_reached="{wingbody_perception_reached}"/>
                        </Sequence>
                    </Fallback>
                </RetryUntilSuccessful>
             </BehaviorTree>
        </root>
    )";
    // clang-format on
    // <IsStopCharge current_task="{current_task}" charge_stop_received="{charge_stop_received}" charge_stop_trigger =
    // "{charge_stop_trigger}"/>

    factory_.registerNodeType<MoveToGoalAction>("MoveToGoal");
    factory_.registerNodeType<GoalReachedCondition>("GoalReached");
    factory_.registerNodeType<IsMoveCondition>("IsMove");
    factory_.registerNodeType<IsManualCondition>("IsManual");
    factory_.registerNodeType<MoveToLiftAction>("MoveToLift");
    factory_.registerNodeType<WaitForManualCondition>("WaitForManual");
    factory_.registerNodeType<IsTurnCondition>("IsTurn");
    factory_.registerNodeType<IsActionCondition>("IsAction");
    factory_.registerNodeType<TurnReachedCondition>("TurnReached");
    factory_.registerNodeType<ActionReachedCondition>("ActionReached");
    factory_.registerNodeType<IsTestCondition>("IsTest");
    factory_.registerNodeType<TestReachedCondition>("TestReached");
    factory_.registerNodeType<IsForkLiftCondition>("IsForkLift");
    factory_.registerNodeType<ForkLiftReachedCondition>("ForkLiftReached");

    factory_.registerNodeType<IsDockingCondition>("IsDocking");
    factory_.registerNodeType<DockingReachedCondition>("DockingReached");
    factory_.registerNodeType<IsUnDockingCondition>("IsUnDocking");
    factory_.registerNodeType<UnDockingReachedCondition>("UnDockingReached");
    factory_.registerNodeType<IsChargingCondition>("IsCharging");
    factory_.registerNodeType<ChargingSuccessCondition>("ChargingSuccess");
    factory_.registerNodeType<IsCheckingCondition>("IsChecking");
    factory_.registerNodeType<CheckingSuccessCondition>("CheckingSuccess");
    factory_.registerNodeType<IsLoadingCondition>("IsLoading");
    factory_.registerNodeType<LoadingSuccessCondition>("LoadingSuccess");
    factory_.registerNodeType<IsWaitCondition>("IsWaitCondition");
    factory_.registerNodeType<WaitSuccessCondition>("WaitSuccessCondition");

    factory_.registerNodeType<IsStopChargeCondition>("IsStopChargeCondition");

    try {
        if (bt_xml_filename.empty()) {
            factory_.registerBehaviorTreeFromText(xml_tree);
        }
        else {
            /// epmty
        }

        blackboard_ = BT::Blackboard::create();
        tree_ = factory_.createTree("MainTree", blackboard_);
        g_bt_engine.reset(new NcBehaviorTreeEngine(tree_, *blackboard_));
        groot2_.reset(new BT::Groot2Publisher(tree_));
        blackboard_->set("previous_task", NaviFra::Task());

        return true;
    }
    catch (std::exception& ex) {
        NLOG(error) << ex.what();
        return false;
    }
}

void TaskManager::TaskCancel()
{
    if (g_bt_engine->isRunning()) {
        NLOG(info) << "requrest stop Command";
        g_bt_engine->cancel();
        blackboard_->set("wait_cancel", true);

        auto shared_queue = std::make_shared<std::deque<Task>>();
        blackboard_->set("tasks", shared_queue);
        Task current_task;
        if (!blackboard_->getAny("current_task")->empty() && blackboard_->get("current_task", current_task)) {
            blackboard_->set("previous_task", current_task);
        }
        blackboard_->set("current_task", NaviFra::Task());
    }
}

void TaskManager::setMode(Mode newMode)
{
    currentMode_ = newMode;

    if (currentMode_ == Mode::MANUAL) {
        pause();
    }
}

}  // namespace NaviFra
