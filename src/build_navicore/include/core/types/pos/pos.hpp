
/*
 * @file	: pos.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: x,y,z,roll,pitch,yaw 정보를 저장
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_POS_HPP_
#define NAVIFRA_POS_HPP_

#include "simplepos/simplepos.hpp"

#include <float.h>

#include <cmath>
#include <iostream>

namespace NaviFra {
class Pos : public SimplePos {
public:
    Pos()
        : SimplePos(){};
    Pos(float f_x, float f_y)
        : SimplePos()
    {
        f_x_m_ = f_x;
        f_y_m_ = f_y;
    };
    Pos(float f_x, float f_y, float f_deg)
        : SimplePos()
    {
        f_x_m_ = f_x;
        f_y_m_ = f_y;
        SetDeg(f_deg);
    };

    enum NODE_TYPE
    {
        NONE = 0,
        POI,
        SPIN_TURN
    };
    enum MISSION_NODE_TYPE
    {
        WAYPOINT = 0,
        CHARGER = 1
    };
    enum LINE_TYPE
    {
        LINE,
        CURVE
    };
    enum DRIVE_TYPE
    {
        FRONT = 1000,
        FRONT_DIAGONAL = 1001,
        REAR = 2000,
        REAR_DIAGONAL = 2001
    };

    struct DriveInfo_t {
        Pos::DRIVE_TYPE e_drive_type = DRIVE_TYPE::FRONT;
        Pos::LINE_TYPE e_curve_type = LINE_TYPE::LINE;

        float f_circle_angle = 0.f;
        float f_circle_pos_x = 0.f;
        float f_circle_pos_y = 0.f;
        float f_linear = 0.5f;
        float f_diagonal_curve = 0.5f;
        bool b_active = true;
        bool b_start_smooth = false;
        bool b_stop_smooth = false;
        bool b_diagonal_align_skip = false;

        bool b_avoid_status = false;
        bool b_avoidance_right = false;
        bool b_avoidance_left = false;
        bool b_lccs = true;
        bool camera_dist_reduction = false;
        bool obstacle_move_reduction = false;
        int n_avoidance_step = 0;
        float f_lane_width = 0.0f;
        int n_avoid_type = 0;
        float f_avoid_speed = 0;

        float f_curve_radius = 0;
        float f_curvature = 0;

        float f_obstacle_margin_front = -1;
        float f_obstacle_margin_rear = -1;
        float f_obstacle_margin_left = -1;
        float f_obstacle_margin_right = -1;

        float f_obstacle_outline_margin_front = -1;
        float f_obstacle_outline_margin_rear = -1;
        float f_obstacle_outline_margin_left = -1;
        float f_obstacle_outline_margin_right = -1;

        float f_target_obstacle_margin_front = -1;
        float f_target_obstacle_margin_rear = -1;
        float f_target_obstacle_margin_left = -1;
        float f_target_obstacle_margin_right = -1;

        float f_move_obstacle_margin_front = -1;
        float f_move_obstacle_margin_rear = -1;
        float f_move_obstacle_margin_left = -1;
        float f_move_obstacle_margin_right = -1;

        float f_docking_check_dist = 0.0;

        int n_camera_index = 0;
        double d_camera_roi_x_m = 0.0;
        double d_camera_roi_y_m = 0.0;
        double d_camera_roi_z_m = 0.0;
        double d_camera_roi2_x_m = 0.0;
        double d_camera_roi2_y_m = 0.0;
        double d_camera_roi2_z_m = 0.0;
        float f_camera_docking_roi_y_m = 0.0;
        float f_camera_docking_roi_z_m = 0.0;
        float f_plt_type_x = 0.0;
        float f_plt_type_y = 0.0;
        float f_heading_bias = 0;
    };

    void SetType(const NODE_TYPE& e_type);

    void SetMissionType(const MISSION_NODE_TYPE& e_type);
    /**
     * @brief Get the Zone Type object
     *
     * @return Pos::NODE_TYPE
     */
    Pos::NODE_TYPE GetType() const;
    /**
     * @brief Get the Mission Zone Type object
     *
     * @return Pos::GetMissionType
     */
    Pos::MISSION_NODE_TYPE GetMissionType() const;

    /**
     * @brief Set the Next Node Name object
     *
     * @param str
     */
    inline void SetNode(const bool& str) { b_is_node_ = str; };

    /**
     * @brief Get the Next Node Name object
     *
     * @return std::string
     */
    inline bool GetNode() { return b_is_node_; };

    /**
     * @brief Set the Next Node Name object
     *
     * @param str
     */
    inline void SetNextNodeName(const std::string& str) { s_next_node_name_ = str; };

    inline void SetNowNodeName(const std::string& str) { s_now_node_name_ = str; };

    inline void SetGoalNodeName(const std::string& str) { s_target_node_name_ = str; };

    inline void SetNextNodeID(const std::string& str) { s_next_node_id_ = str; };

    inline void SetNowNodeID(const std::string& str) { s_now_node_id_ = str; };

    inline void SetGoalNodeID(const std::string& str) { s_target_node_id_ = str; };
    /**
     * @brief Get the Next Node Name object
     *
     * @return std::string
     */
    inline std::string GetNextNodeName() const { return s_next_node_name_; };

    inline std::string GetNowNodeName() const { return s_now_node_name_; };

    inline std::string GetGoalNodeName() const { return s_target_node_name_; };

    inline std::string GetNextNodeID() const { return s_next_node_id_; };

    inline std::string GetNowNodeID() const { return s_now_node_id_; };

    inline std::string GetGoalNodeID() const { return s_target_node_id_; };

    LINE_TYPE GetCurveType() const;
    LINE_TYPE SetCurveType(const LINE_TYPE& curve_type);

    void SetDriveInfo(const DriveInfo_t& s_drive_info);

    /**
     * @brief Get the Drive Info object
     *
     * @return DriveInfo_t&
     */
    DriveInfo_t& GetDriveInfo();

    /**
     * @brief Get the Const Drive Info object
     *
     * @return const DriveInfo_t&
     */
    const DriveInfo_t& GetConstDriveInfo() const;

public:
    DriveInfo_t st_drive_info_;

    virtual inline Pos operator+(const Pos& o_pos) const
    {
        Pos o_result;
        o_result.SetXm(f_x_m_ + o_pos.GetXm());
        o_result.SetYm(f_y_m_ + o_pos.GetYm());
        o_result.SetRad(f_yaw_ + o_pos.GetRad());
        return o_result;
    }

    virtual inline Pos operator-(const Pos& o_pos) const
    {
        Pos o_result;
        o_result.SetXm(f_x_m_ - o_pos.GetXm());
        o_result.SetYm(f_y_m_ - o_pos.GetYm());
        o_result.SetRad(f_yaw_ - o_pos.GetRad());
        return o_result;
    }

    virtual inline Pos operator*(const float f_value) const
    {
        Pos o_result;
        o_result.SetXm(f_x_m_ * f_value);
        o_result.SetYm(f_y_m_ * f_value);
        return o_result;
    }

    virtual inline Pos operator/(const float f_value) const
    {
        Pos o_result;
        if (f_value == 0)
            return o_result;
        o_result.SetXm(f_x_m_ / f_value);
        o_result.SetYm(f_y_m_ / f_value);
        return o_result;
    }
    friend std::ostream& operator<<(std::ostream& o, Pos const& fred);
    Pos inv();

private:
    Pos::NODE_TYPE n_zone_type_ = Pos::NODE_TYPE::NONE;
    Pos::MISSION_NODE_TYPE n_mission_zone_type_ = Pos::MISSION_NODE_TYPE::WAYPOINT;
    LINE_TYPE e_curve_type_ = LINE_TYPE::LINE;

    bool b_is_node_ = false;
    std::string s_next_node_name_;
    std::string s_now_node_name_;
    std::string s_target_node_name_;

    std::string s_next_node_id_;
    std::string s_now_node_id_;
    std::string s_target_node_id_;
};
}  // namespace NaviFra

#endif
