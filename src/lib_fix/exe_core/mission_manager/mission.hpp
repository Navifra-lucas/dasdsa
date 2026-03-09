

#ifndef NAVIFRA_MISSION_HPP_
#define NAVIFRA_MISSION_HPP_

#include "msg_information/parameter_information/param_msg.hpp"
#include "msg_information/sensor_information/sensor_msg.hpp"
#include "pos/pos.hpp"

namespace NaviFra {
class MissionMsg {
public:
    inline void SetPath(const std::vector<NaviFra::Pos>& o_path) { vec_navi_path_ = o_path; }

    inline void SetPath(const std::vector<string>& o_path) { vec_string_path_ = o_path; }

    inline std::vector<Pos> GetPath() const { return vec_navi_path_; }

    inline std::vector<string> GetStringPath() const { return vec_string_path_; }

    inline void SetGoalPos(const NaviFra::Pos& o_pos) { o_goal_pos_ = o_pos; }

    inline Pos GetGoalPos() const { return o_goal_pos_; }

    inline void SetParam(const NaviFra::Parameters_t& st_param) { st_param_ = st_param; }

    inline Parameters_t GetParam() const { return st_param_; }

    inline void SetMissionID(const std::string& s_mission_id) { s_mission_id_ = s_mission_id; }

    inline std::string GetMissionID() const { return s_mission_id_; }

    inline void SetNodeId(const std::string& s_node_id) { s_node_id_ = s_node_id; }

    inline std::string GetNodeId() const { return s_node_id_; }

    inline void SetRobotYawBias(float f_robot_yaw_bias) { f_robot_yaw_bias_ = f_robot_yaw_bias; }

    inline float GetRobotYawBias() const { return f_robot_yaw_bias_; }

    inline void SetNodeName(const std::string& s_node_name) { s_node_name_ = s_node_name; }

    inline std::string GetNodeName() const { return s_node_name_; }

    inline void SetType(const Pos::NODE_TYPE& zone_type) { o_goal_pos_.SetType(zone_type); }

    inline void SetDriveType(const std::vector<int>& vec_drive_type) { vec_drive_type_ = vec_drive_type; }
    inline vector<int> GetDriveType() const { return vec_drive_type_; }


private:
    std::string s_mission_id_;
    std::string s_node_id_;
    std::string s_node_name_;
    NaviFra::Pos o_goal_pos_;
    std::vector<NaviFra::Pos> vec_navi_path_;
    std::vector<string> vec_string_path_;
    NaviFra::Parameters_t st_param_;
    float f_robot_yaw_bias_ = 0.0f;
    vector<int> vec_drive_type_;
};

}  // namespace NaviFra
#endif
