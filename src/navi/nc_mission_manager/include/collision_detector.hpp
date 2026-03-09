#ifndef COLLISION_DETECTION_HPP
#define COLLISION_DETECTION_HPP

#include <chrono>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "pos/pos.hpp"
#include "msg_information/parameter_information/param_msg.hpp"

namespace NaviFra {

class CollisionDetector {
public:
    CollisionDetector() {};
    virtual ~CollisionDetector() {};

    std::array<float,5> GetRobotConfig() const { return {f_robot_max_x_m_, f_robot_min_x_m_, f_robot_max_y_m_, f_robot_min_y_m_, f_robot_radius_m_}; };
    std::array<int,2> GetMapSize() const { return {n_map_size_x_px_, n_map_size_y_px_}; };
    std::array<float,3> GetMapInfo() const { return {f_map_size_x_m_, f_map_size_y_m_, f_map_resolution_}; };

    void SetRobotInfo(float f_robot_max_x, float f_robot_min_x, float f_robot_max_y, float f_robot_min_y, float f_robot_radius);
    void SetMapInfo(const Parameters_t& st_param);
    void SetData(std::shared_ptr< const std::vector<int8_t> > map_ptr, const Pos& o_robot_pos);

    int CheckPathCollision(const std::vector<Pos>& vec_path, float f_margin_m, bool b_backward, int n_start_idx=0) const;
    bool CheckPosCollision(const Pos& o_pos, float f_margin_m, bool b_backward) const;
    bool CheckSpinCollision(const Pos& o_pos, float f_margin_m) const;

    int CalcCost(const std::vector<Pos>& vec_path, float f_margin_m, bool b_backward, int n_start_idx=0) const;

private:
    float f_robot_max_x_m_;
    float f_robot_min_x_m_;
    float f_robot_max_y_m_;
    float f_robot_min_y_m_;
    float f_robot_radius_m_;

    int n_map_size_x_px_;
    int n_map_size_y_px_;
    float f_map_size_x_m_;
    float f_map_size_y_m_;
    float f_map_resolution_;

    mutable std::shared_mutex mtx_;

    Pos o_robot_pos_;

    std::shared_ptr< const std::vector<int8_t> > map_ptr_=nullptr;
};

}  // namespace NaviFra
#endif
