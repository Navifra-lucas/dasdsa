#ifndef NAVIFRA_NAVI_NODE_H_
#define NAVIFRA_NAVI_NODE_H_

#include "core_calculator/core_calculator.hpp"
#include "pos/pos.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace NaviFra {
class NaviEdge {
private:
    float f_dist_robot_2_edge_m_ = 0.f;
    float f_start_to_end_dist_m_ = -1.f;
    int n_start_node_number_ = -1;
    int n_end_node_number_ = -1;

    Pos::DRIVE_TYPE n_drive_from_ = Pos::DRIVE_TYPE::FRONT;
    Pos::DRIVE_TYPE n_drive_to_ = Pos::DRIVE_TYPE::FRONT;

    NaviFra::Pos o_start_pos_;
    NaviFra::Pos o_end_pos_;

    Pos::DriveInfo_t st_info_from_;
    Pos::DriveInfo_t st_info_to_;

public:
    NaviEdge(){};
    virtual ~NaviEdge(){};
    void SetStartNodeNumber(int n_start_node_num);
    int GetStartNodeNumber();
    void SetEndNodeNumber(const int& n_end_node_num);
    int GetEndNodeNumber();
    void SetDistRobot2EdgeM(const float& f_dist_m);
    float GetDistRobot2EdgeM();
    void SetDistM(const float& f_dist_m);
    float GetDistM();
    void SetStartPos(const Pos& o_start_pos);
    Pos GetStartPos();
    void SetEndPos(const Pos& o_end_pos);
    Pos GetEndPos();
    void SetFromDriveDirection(NaviFra::Pos::DRIVE_TYPE dir);
    void SetToDriveDirection(NaviFra::Pos::DRIVE_TYPE dir);
    Pos::DRIVE_TYPE GetFromDriveDirection();
    Pos::DRIVE_TYPE GetToDriveDirection();
    void SetFromDriveInfo(Pos::DriveInfo_t s_drive_info);
    Pos::DriveInfo_t GetFromDriveInfo();
    void SetToDriveInfo(Pos::DriveInfo_t s_drive_info);
    Pos::DriveInfo_t GetToDriveInfo();

    bool operator==(const int& a) const { return n_end_node_number_ == a; }
};

class NaviNode : public Pos {
private:
    std::string str_node_id_ = "";
    std::vector<std::string> vec_from_id_;

    std::string str_node_name_ = "";
    std::vector<NaviEdge> vec_navi_edge_;

    int n_node_number_ = 0;
    float f_dist_m_ = 0.f;

    bool b_alarm_flag_ = false;
    float f_alarm_dist_ = 0.02;
    float f_alarm_deg_ = 1.0;

public:
    NaviNode();
    virtual ~NaviNode(){};
    explicit NaviNode(const Pos& pos);

    bool operator==(const std::string& str_id) const { return str_node_id_ == str_id; }

    bool operator==(const int& n_comp_num) const { return n_node_number_ == n_comp_num; }

    bool operator<(const NaviNode& o_comp) const { return f_dist_m_ < o_comp.f_dist_m_; }
    void SetName(const std::string& stdNodeName);
    void SetID(const std::string& stdNodeID);
    void SetNodeNum(const int& n_node_num);

    void AddEdge(
        const Pos& o_start_pos, const Pos& o_end_pos, int n_start_node_num, int n_end_node_num, Pos::DriveInfo_t s_from_drive_info,
        Pos::DriveInfo_t s_to_drive_info, float f_weight);

    int GetNumber() const;
    void SetDistM(const float& f_dist_m);
    float GetDistM() const;

    std::string GetID() const;
    std::string GetName() const;
    NaviFra::Pos GetPos() const;
    std::vector<NaviEdge> GetAllEdge() const;

    void SetAlarmData(float f_dist, float f_deg);

    vector<float> GetAlarmData();

    bool CheckAlarmNode();

    int n_type = 0;
};
}  // namespace NaviFra
#endif
