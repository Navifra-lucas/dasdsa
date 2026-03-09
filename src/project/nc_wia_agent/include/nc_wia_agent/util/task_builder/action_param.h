#pragma once
#include "common_types.h"
#include "nc_wia_agent/util/yaml_util.h"
#include "pos/pos.hpp"

#include <Poco/JSON/Array.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace NaviFra {

struct BaseParams {
    std::string action_id;
    std::string s_name;

    static BaseParams from(const Poco::JSON::Object::Ptr& work)
    {
        BaseParams p;
        p.action_id = work->getValue<std::string>("action_id");
        std::ostringstream ss;
        ss << std::setw(4) << std::setfill('0') << p.action_id;
        p.s_name = ss.str();

        return p;
    }

    virtual ~BaseParams() = default;

protected:
    static std::unordered_map<std::string, std::string> parseKeyValueParams(Poco::JSON::Array::Ptr params_array)
    {
        std::unordered_map<std::string, std::string> map;
        for (size_t i = 0; i < params_array->size(); ++i) {
            Poco::JSON::Object::Ptr param = params_array->getObject(i);
            map[param->getValue<std::string>("param_name")] = param->getValue<std::string>("value");
        }
        return map;
    }

    //포크리프트용, HACS랑 우리랑 방향이 달라서 만듬..
    static NaviFra::Pos posConvertACS(const NaviFra::Pos& pos)
    {
        NaviFra::Pos o_converted = pos;

        float f_new_deg = pos.GetDeg() + 180.0f;

        while (f_new_deg > 180.0f) {
            f_new_deg -= 360.0f;
        }
        while (f_new_deg < -180.0f) {
            f_new_deg += 360.0f;
        }
        o_converted.SetDeg(f_new_deg);
        NLOG(info) << "Pos Convert! ACS DEG is " << pos.GetDeg() << " -> " << f_new_deg;
        NLOG(info) << "X -> " << o_converted.GetXm() << " / Y -> " << o_converted.GetYm();

        return o_converted;
    }
};

struct MoveParams : public BaseParams {
    std::string node_id;
    NaviFra::Pos o_goal_pos;  //노드 좌표
    float f_max_trans_vel;  //직선 주행 속도

    float f_curve_speed;  //곡선 주행 속도
    float f_passing_dist;  //곡선 주행 반경

    float f_target_lenth;  // obstacle_margin
    float f_target_width;  // obstacle_margin
    int n_payload_level;  //아직 미사용
    int n_drive_type;  //주행 방향

    bool b_lccs_off;  // lccs

    int n_lift_height;  // 신규 추가 필드들
    int n_lidar_field;
    int n_camera_field;
    int n_ossd_field;

    std::vector<float> vec_lccs_field;

    static MoveParams from(const Poco::JSON::Object::Ptr& work)
    {
        MoveParams p;

        static_cast<BaseParams&>(p) = BaseParams::from(work);

        auto params_map = BaseParams::parseKeyValueParams(work->getArray("action_params"));

        Poco::JSON::Array::Ptr args = work->getArray("action_args");
        p.node_id = params_map.at("node_id");

        p.o_goal_pos.SetXm(args->getElement<float>(0));
        p.o_goal_pos.SetYm(args->getElement<float>(1));
        p.o_goal_pos.SetDeg(args->getElement<float>(2) * 180.0 / M_PI);

        p.o_goal_pos = posConvertACS(p.o_goal_pos);

        int n_drive_dir = std::stoi(params_map.at("driving_dir"));
        switch (n_drive_dir) {
            case 0:
                p.n_drive_type = MOVE_BACKWARD;
                break;
            case 1:
                p.n_drive_type = MOVE_FORWARD;
                break;
            case 2:
                p.n_drive_type = CRAB_FORWARD;
                break;
            case 3:
                p.n_drive_type = CRAB_BACKWARD;
                break;
            default:
                p.n_drive_type = MOVE_FORWARD;
                break;
        }
        p.f_max_trans_vel = std::stof(params_map.at("max_trans_vel"));

        p.f_curve_speed = std::stof(params_map.at("curve_speed"));
        p.f_passing_dist = std::stof(params_map.at("passing_dist"));

        p.f_target_lenth = std::stof(params_map.at("target_length"));
        p.f_target_width = std::stof(params_map.at("target_width"));
        p.n_payload_level = std::stoi(params_map.at("payload_level"));

        bool b_lift_usage = params_map.at("lift_usage") == "True" ? true : false;
        p.n_lift_height = b_lift_usage ? std::stoi(params_map.at("height")) : -1;

        p.n_lidar_field = std::stoi(params_map.at("lidar_field"));
        p.n_camera_field = std::stoi(params_map.at("camera_field"));
        p.n_ossd_field = std::stoi(params_map.at("ossd_field"));

        NLOG(info) << p.n_lidar_field;
        if (p.n_lidar_field >= 100) {
            p.b_lccs_off = true;
        }
        else {
            p.b_lccs_off = false;
        }
        p.vec_lccs_field = readYamlParam("LCCS", p.n_lidar_field);
        for (int i = 0; i < p.vec_lccs_field.size(); i++) {
            NLOG(info) << "lccs value : " << p.vec_lccs_field[i];
        }

        return p;
    }
};

struct AddGoalParams : public MoveParams {
    static AddGoalParams from(const Poco::JSON::Object::Ptr& work)
    {
        AddGoalParams p;
        static_cast<MoveParams&>(p) = MoveParams::from(work);
        return p;
    }
};

struct DockingParams : public BaseParams {
    int n_dock_type;  //도킹 종류
    float f_docking_dist;  //도킹 진입 거리
    std::string s_direction;  //도킹 방향
    int n_drive_type;  //주행 방향(task_manager로 해당 데이터 전송)

    NaviFra::Pos o_goal_pos;  //도킹 종료 지점
    NaviFra::Pos o_goal_offset;  //도킹 offset

    std::vector<float> list_f_camera_roi;  //도킹 시 카메라 roi1
    std::vector<float> list_f_camera_roi2;  //도킹 시 카메라 roi2
    std::vector<float> list_f_move_obstacle;  //도킹 move obstacle margin

    int n_pallet_exist;  //파레트 들고있는지

    static DockingParams from(const Poco::JSON::Object::Ptr& work)
    {
        DockingParams p;

        static_cast<BaseParams&>(p) = BaseParams::from(work);

        auto params_map = BaseParams::parseKeyValueParams(work->getArray("action_params"));

        Poco::JSON::Array::Ptr args = work->getArray("action_args");
        p.s_direction = (args->getElement<int>(0) == 1) ? "front" : "rear";
        p.n_dock_type = args->getElement<int>(2);
        p.n_drive_type = (p.s_direction == "front") ? MOVE_FORWARD : MOVE_BACKWARD;

        p.f_docking_dist = std::stof(params_map.at("dock_dist"));

        p.o_goal_pos.SetXm(std::stof(params_map.at("init_x")));
        p.o_goal_pos.SetYm(std::stof(params_map.at("init_y")));
        p.o_goal_pos.SetDeg(std::stof(params_map.at("init_th")) * 180.0 / M_PI);

        p.o_goal_pos = posConvertACS(p.o_goal_pos);

        p.o_goal_offset.SetXm(std::stof(params_map.at("ep_x")));
        p.o_goal_offset.SetYm(std::stof(params_map.at("ep_y")));
        p.o_goal_offset.SetDeg(std::stof(params_map.at("ep_th")) * 180.0 / M_PI);

        p.o_goal_offset = posConvertACS(p.o_goal_offset);

        p.list_f_camera_roi.emplace_back(std::stof(params_map.at("camera_detect_width")));
        p.list_f_camera_roi.emplace_back(std::stof(params_map.at("camera_detect_height")));
        p.list_f_camera_roi2.emplace_back(std::stof(params_map.at("camera_detect_width2")));

        p.list_f_move_obstacle.emplace_back(std::stof(params_map.at("footprint_f")));
        p.list_f_move_obstacle.emplace_back(std::stof(params_map.at("footprint_b")));
        p.list_f_move_obstacle.emplace_back(std::stof(params_map.at("footprint_l")));
        p.list_f_move_obstacle.emplace_back(std::stof(params_map.at("footprint_r")));

        p.n_pallet_exist = std::stoi(params_map.at("pallet_exist"));

        return p;
    }
};

struct UndockingParams : public BaseParams {
    NaviFra::Pos o_goal_pos;

    int n_drive_type;

    float f_docking_dist;
    float f_dockout_speed;

    std::vector<float> list_f_target_obstacle_margin;

    static UndockingParams from(const Poco::JSON::Object::Ptr& work)
    {
        UndockingParams p;
        static_cast<BaseParams&>(p) = BaseParams::from(work);

        auto params_map = BaseParams::parseKeyValueParams(work->getArray("action_params"));

        Poco::JSON::Array::Ptr args = work->getArray("action_args");
        p.f_docking_dist = args->getElement<float>(0);
        p.n_drive_type = p.f_docking_dist > 0 ? MOVE_FORWARD : MOVE_BACKWARD;

        p.o_goal_pos.SetXm(std::stof(params_map.at("init_x")));
        p.o_goal_pos.SetYm(std::stof(params_map.at("init_y")));
        p.o_goal_pos.SetDeg(std::stof(params_map.at("init_th")) * 180.0 / M_PI);

        p.o_goal_pos = posConvertACS(p.o_goal_pos);

        p.f_dockout_speed = 0.2;

        p.list_f_target_obstacle_margin.emplace_back(std::stof(params_map.at("target_obstacle_f")));
        p.list_f_target_obstacle_margin.emplace_back(std::stof(params_map.at("target_obstacle_b")));
        p.list_f_target_obstacle_margin.emplace_back(std::stof(params_map.at("target_obstacle_l")));
        p.list_f_target_obstacle_margin.emplace_back(std::stof(params_map.at("target_obstacle_r")));

        return p;
    }
};

struct LiftParams : public BaseParams {
    int n_lift_state;

    static LiftParams from(const Poco::JSON::Object::Ptr& work)
    {
        LiftParams p;

        static_cast<BaseParams&>(p) = BaseParams::from(work);

        Poco::JSON::Array::Ptr args = work->getArray("action_args");
        p.n_lift_state = (args->getElement<int>(0));

        return p;
    }
};

struct StandbyParams : public BaseParams {
    int n_standby_time;
    std::string s_standby_id;

    static StandbyParams from(const Poco::JSON::Object::Ptr& work)
    {
        StandbyParams p;

        static_cast<BaseParams&>(p) = BaseParams::from(work);

        auto params_map = BaseParams::parseKeyValueParams(work->getArray("action_params"));

        Poco::JSON::Array::Ptr args = work->getArray("action_args");
        p.n_standby_time = args->getElement<int>(0);
        p.s_standby_id = params_map.at("id");

        return p;
    }
};

struct SpinParams : public BaseParams {
    float f_goal_deg;

    static SpinParams from(const Poco::JSON::Object::Ptr& work)
    {
        SpinParams p;
        static_cast<BaseParams&>(p) = BaseParams::from(work);

        auto params_map = BaseParams::parseKeyValueParams(work->getArray("action_params"));

        p.f_goal_deg = std::stof(params_map.at("init_th")) * 180.0 / M_PI;
        p.f_goal_deg += 180.0;
        if (p.f_goal_deg > 360.0) {
            p.f_goal_deg -= 360.0;
        }

        return p;
    }
};

struct ForkLiftParams : public BaseParams {
    int n_drive_type;  // 0 : wingbody perception, 1 : up , -1 : down, 2 : 2단..?

    std::string s_current_node_id;
    std::string s_target_node_id;

    NaviFra::Pos o_current_pos;
    NaviFra::Pos o_target_pos;
    // NaviFra::Pos o_offset_pos;

    int n_rack_level;
    int n_target_level;
    int n_target_height = 1;
    int n_rack_type;  //어디 도킹인지.. 평치, 랙, 윙바디

    int n_pallet_size_width;
    int n_pallet_size_length;
    int n_pallet_size_height;

    int n_pallet_type;  //어떤 팔레트인지..

    static ForkLiftParams from(const Poco::JSON::Object::Ptr& work)
    {
        ForkLiftParams p;
        static_cast<BaseParams&>(p) = BaseParams::from(work);

        Poco::JSON::Array::Ptr args = work->getArray("action_args");
        if (!args || args->size() < 1) {
            throw std::runtime_error("ForkliftParams: action_args missing");
        }
        p.n_drive_type = (args->getElement<int>(0));

        auto params_map = BaseParams::parseKeyValueParams(work->getArray("action_params"));
        auto get = [&](const std::string& key) -> std::string {
            if (!params_map.count(key)) {
                throw std::runtime_error("ForkliftParams missing field: " + key);
            }
            return params_map.at(key);
        };

        p.s_current_node_id = params_map.at("current_node_id");
        p.s_target_node_id = params_map.at("target_node_id");

        try {
            p.o_current_pos.SetXm(std::stof(get("current_x")));
            p.o_current_pos.SetYm(std::stof(get("current_y")));
            p.o_current_pos.SetDeg(std::stof(get("current_theta")) * 180.0 / M_PI);

            p.o_current_pos = posConvertACS(p.o_current_pos);

            p.o_target_pos.SetXm(std::stof(get("target_x")));
            p.o_target_pos.SetYm(std::stof(get("target_y")));
            p.o_target_pos.SetDeg(std::stof(get("target_theta")) * 180.0 / M_PI);

            // p.o_offset_pos.SetXm(std::stof(get("offset_x")));
            // p.o_offset_pos.SetYm(std::stof(get("offset_y")));
            // p.o_offset_pos.SetDeg(std::stof(get("offset_theta")) * 180.0 / M_PI);

            p.o_target_pos = posConvertACS(p.o_target_pos);

            p.n_rack_level = static_cast<int>(std::stof(get("rack_level")));
            p.n_target_level = static_cast<int>(std::stof(get("target_level")));
            p.n_rack_type = static_cast<int>(std::stof(get("rack_type")));

            if (p.n_drive_type == 0) {
                p.n_drive_type = 6;  // 임시... dcs에선 wingbody_perception을 6으로 사용중...
            }

            if (p.n_target_level == 2 && p.n_rack_type == 1) {
                p.n_rack_type = 3;
                if (p.n_drive_type == 1) {
                    p.n_drive_type = 2;
                }
            }
            else if(p.n_rack_type == 2){
                p.n_drive_type = 3;
            }

            p.n_pallet_size_width = static_cast<int>(std::stof(get("pallet_size_width")));
            p.n_pallet_size_length = static_cast<int>(std::stof(get("pallet_size_length")));
            p.n_pallet_size_height = static_cast<int>(std::stof(get("pallet_size_height")));

            if (p.n_pallet_size_width == 2100 && p.n_pallet_size_length == 2100 && p.n_pallet_size_height == 1100) {
                p.n_pallet_type = 0;
            }
            else if (p.n_pallet_size_width == 1960 && p.n_pallet_size_length == 1500 && p.n_pallet_size_height == 1900) {
                p.n_pallet_type = 1;
            }
            else {
                std::ostringstream oss;
                oss << "Unsupported pallet size detected "
                    << "(width=" << p.n_pallet_size_width << ", length=" << p.n_pallet_size_length << ", height=" << p.n_pallet_size_height
                    << ")";
                throw std::runtime_error(oss.str());
            }
        }
        catch (...) {
            throw std::runtime_error("ForkliftParams parse error (invalid numeric)");
        }

        return p;
    }
};
}  // namespace NaviFra
