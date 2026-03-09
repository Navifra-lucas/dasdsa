#include "pos.hpp"
namespace NaviFra {

Pos::LINE_TYPE Pos::GetCurveType() const
{
    return e_curve_type_;
}
Pos::LINE_TYPE Pos::SetCurveType(const Pos::LINE_TYPE& curve_type)
{
    return e_curve_type_ = curve_type;
}

void Pos::SetType(const Pos::NODE_TYPE& e_type)
{
    n_zone_type_ = e_type;
}

void Pos::SetMissionType(const Pos::MISSION_NODE_TYPE& e_type)
{
    n_mission_zone_type_ = e_type;
}

Pos::NODE_TYPE Pos::GetType() const
{
    return n_zone_type_;
}

Pos::MISSION_NODE_TYPE Pos::GetMissionType() const
{
    return n_mission_zone_type_;
}

void Pos::SetDriveInfo(const Pos::DriveInfo_t& s_drive_info)
{
    st_drive_info_ = s_drive_info;
}
Pos::DriveInfo_t& Pos::GetDriveInfo()
{
    return st_drive_info_;
}
const Pos::DriveInfo_t& Pos::GetConstDriveInfo() const
{
    return st_drive_info_;
}

Pos Pos::inv()
{
    Pos b_to_a;
    b_to_a.f_x_m_ = -this->GetXm() * cosf(this->GetRad()) - this->GetYm() * sinf(this->GetRad());
    b_to_a.f_y_m_ = this->GetXm() * sinf(this->GetRad()) - this->GetYm() * cosf(this->GetRad());
    b_to_a.f_yaw_ = ((-1) * this->GetRad());

    float two_pi = 2 * M_PI;
    auto rad = b_to_a.f_yaw_;
    rad -= two_pi * std::floor((rad + M_PI) * (1. / (two_pi)));
    b_to_a.f_yaw_ = rad;

    return b_to_a;
}

std::ostream& operator<<(std::ostream& o, Pos const& pose2d)
{
    o.precision(4);
    return o << "[" << pose2d.GetXm() << "," << pose2d.GetYm() << "," << pose2d.GetDeg() << "]"
             << "\n";
}
}  // namespace NaviFra
