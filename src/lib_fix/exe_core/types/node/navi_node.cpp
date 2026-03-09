
#include "navi_node.hpp"

using namespace std;
namespace NaviFra {
void NaviEdge::SetStartNodeNumber(int n_start_node_num) {
  n_start_node_number_ = n_start_node_num;
}

int NaviEdge::GetStartNodeNumber() { return n_start_node_number_; }

void NaviEdge::SetEndNodeNumber(const int &n_end_node_num) {
  n_end_node_number_ = n_end_node_num;
}

int NaviEdge::GetEndNodeNumber() { return n_end_node_number_; }

void NaviEdge::SetDistRobot2EdgeM(const float &f_dist_m) {
  f_dist_robot_2_edge_m_ = f_dist_m;
}

float NaviEdge::GetDistRobot2EdgeM() { return f_dist_robot_2_edge_m_; }

void NaviEdge::SetDistM(const float &f_dist_m) {
  f_start_to_end_dist_m_ = f_dist_m;
}

float NaviEdge::GetDistM() { return f_start_to_end_dist_m_; }

void NaviEdge::SetStartPos(const Pos &o_start_pos) {
  o_start_pos_ = o_start_pos;
}

Pos NaviEdge::GetStartPos() { return o_start_pos_; }

void NaviEdge::SetEndPos(const Pos &o_end_pos) { o_end_pos_ = o_end_pos; }

Pos NaviEdge::GetEndPos() { return o_end_pos_; }

void NaviEdge::SetFromDriveDirection(NaviFra::Pos::DRIVE_TYPE dir) {
  n_drive_from_ = dir;
}

void NaviEdge::SetToDriveDirection(NaviFra::Pos::DRIVE_TYPE dir) {
  n_drive_to_ = dir;
}

Pos::DRIVE_TYPE NaviEdge::GetFromDriveDirection() { return n_drive_from_; }

Pos::DRIVE_TYPE NaviEdge::GetToDriveDirection() { return n_drive_to_; }

void NaviEdge::SetFromDriveInfo(Pos::DriveInfo_t s_drive_info) {
  st_info_from_ = s_drive_info;
}

Pos::DriveInfo_t NaviEdge::GetFromDriveInfo() { return st_info_from_; }

void NaviEdge::SetToDriveInfo(Pos::DriveInfo_t s_drive_info) {
  st_info_to_ = s_drive_info;
}

Pos::DriveInfo_t NaviEdge::GetToDriveInfo() { return st_info_to_; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

NaviNode::NaviNode() : Pos() {
  vec_navi_edge_.reserve(16);
  str_node_id_.reserve(36);
}

NaviNode::NaviNode(const Pos &pos) : Pos(pos) { vec_navi_edge_.reserve(16); }

float NaviNode::GetDistM() const { return f_dist_m_; }

void NaviNode::SetDistM(const float &f_dist_m) { f_dist_m_ = f_dist_m; }

void NaviNode::SetID(const std::string &s_node_id) { str_node_id_ = s_node_id; }

void NaviNode::SetNodeNum(const int &n_node_num) {
  n_node_number_ = n_node_num;
}

void NaviNode::SetName(const std::string &s_node_name) {
  str_node_name_ = s_node_name;
}

int NaviNode::GetNumber() const { return n_node_number_; }

string NaviNode::GetID() const { return str_node_id_; }

string NaviNode::GetName() const { return str_node_name_; }

Pos NaviNode::GetPos() const {
  NaviFra::Pos o_pos;
  o_pos.SetXm(Pos::GetXm());
  o_pos.SetYm(Pos::GetYm());
  o_pos.SetDeg(Pos::GetDeg());
  return o_pos;
}

void NaviNode::AddEdge(const Pos &o_start_pos, const Pos &o_end_pos,
                       int n_start_node_num, int n_end_node_num,
                       Pos::DriveInfo_t s_from_drive_info,
                       Pos::DriveInfo_t s_to_drive_info, float f_weight) {
  //노드간의 연관관계를 만들어 주는 함수

  NaviEdge o_edge;
  o_edge.SetDistM(f_weight);
  o_edge.SetStartPos(o_start_pos);
  o_edge.SetStartNodeNumber(n_start_node_num);
  o_edge.SetEndPos(o_end_pos);
  o_edge.SetEndNodeNumber(n_end_node_num);
  o_edge.SetFromDriveInfo(s_from_drive_info);
  o_edge.SetToDriveInfo(s_to_drive_info);
  vec_navi_edge_.emplace_back(o_edge);
}

vector<NaviEdge> NaviNode::GetAllEdge() const { return vec_navi_edge_; }

void NaviNode::SetAlarmData(float f_dist, float f_deg) {
  b_alarm_flag_ = true;
  f_alarm_dist_ = f_dist;
  f_alarm_deg_ = f_deg;
}

bool NaviNode::CheckAlarmNode() { return b_alarm_flag_; }

vector<float> NaviNode::GetAlarmData() {
  vector<float> vec_data;
  vec_data.push_back(f_alarm_dist_);
  vec_data.push_back(f_alarm_deg_);
  return vec_data;
}
} // namespace NaviFra
