#include "nc_task_manager/nc_task_manager_pch.h"

#include <nc_task_manager/data/nc_task.h>

#include <ostream>

using namespace NaviFra;
Task::Task()
{
}

Task::Task(Poco::JSON::Object::Ptr obj)
{
    if (!obj->isNull("start_node")) {
        start_node_ = obj->get("start_node").convert<std::string>();
    }

    if (!obj->isNull("end_node")) {
        end_node_ = obj->get("end_node").convert<std::string>();
    }

    if (!obj->isNull("start_node_name")) {
        start_node_name_ = obj->get("start_node_name").convert<std::string>();
    }

    if (!obj->isNull("end_node_name")) {
        end_node_name_ = obj->get("end_node_name").convert<std::string>();
    }

    if (!obj->isNull("docking_type")) {
        docking_type_ = obj->get("docking_type").convert<std::string>();
    }

    if (!obj->isNull("direction")) {
        direction_ = obj->get("direction").convert<std::string>();
    }

    if (!obj->isNull("velocity")) {
        velocity_ = obj->get("velocity").convert<float>();
    }

    if (!obj->isNull("finish_angle")) {
        finish_angle_ = obj->get("finish_angle").convert<float>();
    }
    else {
        finish_angle_ = 999;
    }

    if (!obj->isNull("is_align")) {
        is_align_ = obj->get("is_align").convert<bool>();
    }
    else {
        is_align_ = false;
    }

    if (!obj->isNull("type")) {
        type_ = obj->get("type").convert<std::string>();
        if (type_ == "turn") {
            type_ = "move";
        }
    }

    if (!obj->isNull("uuid")) {
        uuid_ = obj->get("uuid").convert<std::string>();
    }

    if (!obj->isNull("subtask_id")) {
        subtask_id_ = obj->get("subtask_id").convert<std::string>();
    }

    if (obj->has("data")) {
        data_ = new Poco::JSON::Object(*obj->getObject("data"));
    }

    if (!obj->isNull("level")) {
        level_ = obj->get("level").convert<int>();
    }
    else {
        level_ = -1;
    }

    if (!obj->isNull("count")) {
        count_ = obj->get("count").convert<int>();
    }
    else {
        count_ = -1;
    }

    if (!obj->isNull("b_arrive_align")) {
        b_arrive_align_ = obj->get("b_arrive_align").convert<bool>();
    }
    else {
        b_arrive_align_ = false;
    }

    if (!obj->isNull("charger_id")) {
        charger_id_ = obj->get("charger_id").convert<int>();
    }
    else {
        charger_id_ = -1;
    }

    if (!obj->isNull("wait_seconds")) {
        wait_seconds_ = obj->get("wait_seconds").convert<int>();
    }
    else {
        wait_seconds_ = -1;
    }

    if (!obj->isNull("wait_id")) {
        wait_id_ = obj->get("wait_id").convert<std::string>();
    }
    else {
        wait_id_ = "";
    }

    if (!obj->isNull("target_node")) {
        target_node_ = obj->get("target_node").convert<std::string>();
    }

    if (!obj->isNull("drive_type")) {
        drive_type_ = obj->get("drive_type").convert<int>();
    }
    else {
        drive_type_ = -1;
    }

    if (!obj->isNull("docking_dist")) {
        docking_dist_ = obj->get("docking_dist").convert<float>();
    }
    else {
        docking_dist_ = -1;
    }

    if (!obj->isNull("pallet_exist")) {
        pallet_exist_ = obj->get("pallet_exist").convert<int>();
    }
    else {
        pallet_exist_ = 0;
    }

    if (!obj->isNull("goal_offset_x")) {
        goal_offset_x_ = obj->get("goal_offset_x").convert<float>();
    }
    else {
        goal_offset_x_ = 0;
    }

    if (!obj->isNull("goal_offset_y")) {
        goal_offset_y_ = obj->get("goal_offset_y").convert<float>();
    }
    else {
        goal_offset_y_ = 0;
    }

    if (!obj->isNull("goal_offset_deg")) {
        goal_offset_deg_ = obj->get("goal_offset_deg").convert<float>();
    }
    else {
        goal_offset_deg_ = 0;
    }

    // "path_nodes" 처리 (배열)
    if (!obj->isNull("path_nodes")) {
        Poco::JSON::Array::Ptr path_nodes_array = obj->getArray("path_nodes");
        if (path_nodes_array) {  // 배열이 유효한지 확인
            for (size_t i = 0; i < path_nodes_array->size(); ++i) {
                std::string node = path_nodes_array->getElement<std::string>(i);
                path_nodes_.push_back(node);  // 벡터에 추가
            }
        }
    }

    if (!obj->isNull("list_f_camera_roi")) {
        Poco::JSON::Array::Ptr list_f_camera_roi = obj->getArray("list_f_camera_roi");
        if (list_f_camera_roi) {  // 배열이 유효한지 확인
            for (size_t i = 0; i < list_f_camera_roi->size(); ++i) {
                float value = list_f_camera_roi->optElement<float>(i, 0.0f);
                NLOG(info) << "list_f_camera_roi : " << value;
                list_f_camera_roi_.push_back(value);  // 벡터에 추가
            }
        }
    }

    if (!obj->isNull("list_f_camera_roi2")) {
        Poco::JSON::Array::Ptr list_f_camera_roi2 = obj->getArray("list_f_camera_roi2");
        if (list_f_camera_roi2) {  // 배열이 유효한지 확인
            for (size_t i = 0; i < list_f_camera_roi2->size(); ++i) {
                float value = list_f_camera_roi2->optElement<float>(i, 0.0f);
                NLOG(info) << "list_f_camera_roi2 : " << value;
                list_f_camera_roi2_.push_back(value);  // 벡터에 추가
            }
        }
    }

    if (!obj->isNull("list_f_move_obstacle")) {
        Poco::JSON::Array::Ptr list_f_move_obstacle = obj->getArray("list_f_move_obstacle");
        if (list_f_move_obstacle) {  // 배열이 유효한지 확인
            for (size_t i = 0; i < list_f_move_obstacle->size(); ++i) {
                float value = list_f_move_obstacle->optElement<float>(i, 0.0f);
                NLOG(info) << "list_f_move_obstacle : " << value;
                list_f_move_obstacle_.push_back(value);  // 벡터에 추가
            }
        }
    }

    if (!obj->isNull("list_f_obstacle")) {
        Poco::JSON::Array::Ptr list_f_obstacle = obj->getArray("list_f_obstacle");
        if (list_f_obstacle) {  // 배열이 유효한지 확인
            for (size_t i = 0; i < list_f_obstacle->size(); ++i) {
                float value = list_f_obstacle->optElement<float>(i, 0.0f);
                NLOG(info) << "list_f_obstacle : " << value;
                list_f_obstacle_.push_back(value);  // 벡터에 추가
            }
        }
    }
    if (!obj->isNull("list_f_target_obstacle")) {
        Poco::JSON::Array::Ptr list_f_target_obstacle = obj->getArray("list_f_target_obstacle");
        if (list_f_target_obstacle) {  // 배열이 유효한지 확인
            for (size_t i = 0; i < list_f_target_obstacle->size(); ++i) {
                float value = list_f_target_obstacle->optElement<float>(i, 0.0f);
                NLOG(info) << "list_f_target_obstacle : " << value;
                list_f_target_obstacle_.push_back(value);  // 벡터에 추가
            }
        }
    }

    // "vec_move_data" 처리 (배열)
    if (!obj->isNull("vec_move_data")) {
        Poco::JSON::Array::Ptr vec_move_data_array = obj->getArray("vec_move_data");
        if (vec_move_data_array) {  // 배열이 유효한지 확인
            for (size_t i = 0; i < vec_move_data_array->size(); ++i) {
                Poco::JSON::Object::Ptr move_obj = vec_move_data_array->getObject(i);
                NaviFra::Task::MoveData move_data;

                // string 필드 체크
                move_data.s_id = move_obj->optValue<std::string>("s_id", "");
                move_data.s_name = move_obj->optValue<std::string>("s_name", "");

                // int 필드 체크
                move_data.n_drive_type = move_obj->optValue<int>("n_drive_type", 0);
                move_data.n_avoid_type = move_obj->optValue<int>("n_avoid_type", 0);

                // float 필드 체크
                move_data.f_speed_ms = move_obj->optValue<float>("f_speed_ms", 0.0f);
                move_data.f_x_m = move_obj->optValue<float>("f_x_m", 0.0f);
                move_data.f_y_m = move_obj->optValue<float>("f_y_m", 0.0f);
                move_data.f_angle_deg = move_obj->optValue<float>("f_angle_deg", 0.0f);
                move_data.f_curve_radius = move_obj->optValue<float>("f_curve_radius", 0.0f);
                move_data.f_curvature = move_obj->optValue<float>("f_curvature", 0.0f);
                move_data.f_avoid_lanewidth = move_obj->optValue<float>("f_avoid_lanewidth", 0.0f);
                move_data.f_avoid_speed_ms = move_obj->optValue<float>("f_avoid_speed_ms", 0.0f);
                move_data.f_diagonal_heading_bias = move_obj->optValue<float>("f_diagonal_heading_bias", 0.0f);
                move_data.f_arrive_boundary_dist = move_obj->optValue<float>("f_arrive_boundary_dist", 0.0f);
                move_data.f_arrive_boundary_deg = move_obj->optValue<float>("f_arrive_boundary_deg", 0.0f);
                move_data.f_side_check_margin_reduction_left = move_obj->optValue<float>("f_side_check_margin_reduction_left", 0.0f);
                move_data.f_side_check_margin_reduction_right = move_obj->optValue<float>("f_side_check_margin_reduction_right", 0.0f);

                // bool 필드 체크
                move_data.b_start_pause = move_obj->optValue<bool>("b_start_pause", false);
                move_data.b_start_quick = move_obj->optValue<bool>("b_start_quick", false);
                move_data.b_stop_quick = move_obj->optValue<bool>("b_stop_quick", false);
                move_data.b_loaded = move_obj->optValue<bool>("b_loaded", false);
                move_data.b_arrive_align = move_obj->optValue<bool>("b_arrive_align", false);
                move_data.b_diagonal_align_skip = move_obj->optValue<bool>("b_diagonal_align_skip", false);
                move_data.b_obstacle_side_check = move_obj->optValue<bool>("b_obstacle_side_check", false);
                move_data.b_lccs_off = move_obj->optValue<bool>("b_lccs_off", false);

                // std::vector<float> 필드 처리 함수화 (안전 처리)
                auto getFloatVector = [&](Poco::JSON::Object::Ptr o, const std::string& key) {
                    std::vector<float> result(4, 0.0f);  // default [0,0,0,0]
                    if (o->has(key)) {
                        Poco::JSON::Array::Ptr arr = o->getArray(key);
                        if (arr) {
                            size_t min_size = std::min<size_t>(arr->size(), 4);
                            for (size_t idx = 0; idx < min_size; ++idx) {
                                result[idx] = arr->optElement<float>(idx, 0.0f);
                            }
                        }
                    }
                    return result;
                };

                // vector 필드 안전 처리
                move_data.list_f_target_obstacle_margin = getFloatVector(move_obj, "list_f_target_obstacle_margin");
                move_data.list_f_move_obstacle_margin = getFloatVector(move_obj, "list_f_move_obstacle_margin");
                move_data.list_f_obstacle_margin = getFloatVector(move_obj, "list_f_obstacle_margin");

                // 최종 struct vector에 추가
                vec_move_data_.push_back(move_data);
            }
        }
    }

    if (!obj->isNull("fork_lift_data")) {
        Poco::JSON::Array::Ptr fork_lift_data_array = obj->getArray("fork_lift_data");
        if (fork_lift_data_array && fork_lift_data_array->size() != 0) {
            Poco::JSON::Object::Ptr lift_obj = fork_lift_data_array->getObject(0);
            NaviFra::Task::ForkLiftData lift_data;

            lift_data.s_current_node_id = lift_obj->optValue<std::string>("current_node_id", "");
            lift_data.s_target_node_id = lift_obj->optValue<std::string>("target_node_id", "");

            if (lift_obj->has("current_pos")) {
                Poco::JSON::Object::Ptr current_pos_obj = lift_obj->getObject("current_pos");
                lift_data.f_current_x = current_pos_obj->optValue<float>("x", 0.0f);
                lift_data.f_current_y = current_pos_obj->optValue<float>("y", 0.0f);
                lift_data.f_current_deg = current_pos_obj->optValue<float>("deg", 0.0f);
            }

            if (lift_obj->has("target_pos")) {
                Poco::JSON::Object::Ptr target_pos_obj = lift_obj->getObject("target_pos");
                lift_data.f_target_x = target_pos_obj->optValue<float>("x", 0.0f);
                lift_data.f_target_y = target_pos_obj->optValue<float>("y", 0.0f);
                lift_data.f_target_deg = target_pos_obj->optValue<float>("deg", 0.0f);
            }

            if (lift_obj->has("offset_pos")) {
                Poco::JSON::Object::Ptr offset_pos_obj = lift_obj->getObject("offset_pos");
                lift_data.f_current_x += offset_pos_obj->optValue<float>("x", 0.0f);
                lift_data.f_current_y += offset_pos_obj->optValue<float>("y", 0.0f);
                lift_data.f_current_deg += offset_pos_obj->optValue<float>("deg", 0.0f);
                lift_data.f_target_x += offset_pos_obj->optValue<float>("x", 0.0f);
                lift_data.f_target_y += offset_pos_obj->optValue<float>("y", 0.0f);
                lift_data.f_target_deg += offset_pos_obj->optValue<float>("deg", 0.0f);
            }

            lift_data.n_rack_level = lift_obj->optValue<int>("rack_level", 0);
            lift_data.n_rack_type = lift_obj->optValue<int>("rack_type", 0);
            lift_data.n_target_level = lift_obj->optValue<int>("target_level", 0);
            lift_data.n_target_height = lift_obj->optValue<int>("target_height", 0);
            lift_data.n_drive_type = lift_obj->optValue<int>("drive_type", 0);
            lift_data.n_pallet_type = lift_obj->optValue<int>("pallet_type", 0);

            fork_lift_data_ = lift_data;
        }
    }
}

Task::~Task()
{
}

std::string Task::toString()
{
    return "";
}

Poco::JSON::Object Task::toObject()
{
    Poco::JSON::Object obj;
    obj.set("uuid", uuid_);
    obj.set("type", type_);

    // pos list
    obj.set("path_nodes", path_nodes_);
    // obj.set("vec_move_data", vec_move_data_);

    // topological map
    // node target
    obj.set("target_node", target_node_);

    // node list
    obj.set("start_node", start_node_);
    obj.set("end_node", end_node_);
    obj.set("finish_angle", finish_angle_);
    obj.set("is_align", is_align_);
    obj.set("start_node_name", start_node_name_);
    obj.set("end_node_name", end_node_name_);
    obj.set("docking_type", docking_type_);
    obj.set("b_arrive_align", b_arrive_align_);
    obj.set("direction", direction_);
    obj.set("velocity", velocity_);
    obj.set("level", level_);
    obj.set("count", count_);
    obj.set("charger_id", charger_id_);
    obj.set("wait_seconds", wait_seconds_);
    obj.set("wait_id", wait_id_);

    return obj;
}

const std::string& Task::uuid() const
{
    return uuid_;
}
const std::string& Task::type() const
{
    return type_;
}
const std::string& Task::docking_type() const
{
    return docking_type_;
}
const std::string& Task::direction() const
{
    return direction_;
}
double Task::velocity()
{
    return velocity_;
}
const std::string& Task::start_node_name() const
{
    return start_node_name_;
}
const std::string& Task::end_node_name() const
{
    return end_node_name_;
}
const std::string& Task::startNode() const
{
    return start_node_;
}
const std::string& Task::endNode() const
{
    return end_node_;
}
const std::string& Task::targetNode() const
{
    return target_node_;
}
double Task::finishAngle()
{
    return finish_angle_;
}
bool Task::isAlign()
{
    return is_align_;
}
bool Task::arrvieAlign()
{
    return b_arrive_align_;
}
const std::vector<Task::MoveData>& Task::moveData() const
{
    return vec_move_data_;
}

const Task::ForkLiftData& Task::forkLiftData() const
{
    return fork_lift_data_;
}

std::string Task::print()
{
    return Poco::format(
        "TASK INFO UUID [ %s ] TYPE [ %s ] S_NODE [ %s ] E_NODE [ %s ] F_ANGLE [ %.3hf ] B_IS_ALIGN [ %d ]", uuid_, type_, start_node_,
        end_node_, finish_angle_, is_align_ ? "true" : "false");
}
int Task::level()
{
    return level_;
}

int Task::count()
{
    return count_;
}

int Task::charger_id()
{
    return charger_id_;
}

const std::vector<float>& Task::camera_roi() const
{
    return list_f_camera_roi_;
}

const std::vector<float>& Task::camera_roi2() const
{
    return list_f_camera_roi2_;
}

const std::vector<float>& Task::move_obstacle() const
{
    return list_f_move_obstacle_;
}

const std::vector<float>& Task::obstacle() const
{
    return list_f_obstacle_;
}

const std::vector<float>& Task::target_obstacle() const
{
    return list_f_target_obstacle_;
}

int Task::wait_seconds()
{
    return wait_seconds_;
}

std::string Task::wait_id()
{
    return wait_id_;
}

int Task::drive_type()
{
    return drive_type_;
}

float Task::docking_dist()
{
    return docking_dist_;
}

int Task::pallet_exist()
{
    return pallet_exist_;
}

std::vector<float> Task::goal_offset()
{
    goal_offset_.clear();
    goal_offset_.emplace_back(goal_offset_x_);
    goal_offset_.emplace_back(goal_offset_y_);
    goal_offset_.emplace_back(goal_offset_deg_);
    return goal_offset_;
}