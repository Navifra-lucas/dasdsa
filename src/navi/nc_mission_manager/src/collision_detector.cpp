
#include "collision_detector.hpp"

#include "core/util/logger.hpp"

using namespace std;

namespace NaviFra {

void CollisionDetector::SetRobotInfo(
    float f_robot_max_x, float f_robot_min_x, float f_robot_max_y, float f_robot_min_y, float f_robot_radius)
{
    f_robot_max_x_m_ = f_robot_max_x;
    f_robot_min_x_m_ = f_robot_min_x;
    f_robot_max_y_m_ = f_robot_max_y;
    f_robot_min_y_m_ = f_robot_min_y;
    f_robot_radius_m_ = f_robot_radius;
    LOG_INFO(
        "[CollisionDetector::SetRobotInfo] X (%f, %f), Y (%f, %f), R (%f)", f_robot_max_x_m_, f_robot_min_x_m_, f_robot_min_y_m_,
        f_robot_max_y_m_, f_robot_radius_m_);
}

void CollisionDetector::SetMapInfo(const Parameters_t& st_param)
{
    f_map_size_x_m_ = st_param.st_local_mission_param.map_size_x_m;
    f_map_size_y_m_ = st_param.st_local_mission_param.map_size_y_m;
    f_map_resolution_ = st_param.st_local_mission_param.map_res_m;
    n_map_size_x_px_ = f_map_size_x_m_ / f_map_resolution_;
    n_map_size_y_px_ = f_map_size_y_m_ / f_map_resolution_;
    LOG_INFO(
        "[CollisionDetector::SetMapInfo] x_m %f, y_m %f, res %f, x_px %d, y_px %d", f_map_size_x_m_, f_map_size_y_m_, f_map_resolution_,
        n_map_size_x_px_, n_map_size_y_px_);
}

void CollisionDetector::SetData(std::shared_ptr<const std::vector<int8_t>> map_ptr, const Pos& o_robot_pos)
{
    std::unique_lock lock(mtx_);
    map_ptr_ = map_ptr;
    o_robot_pos_ = o_robot_pos;
}

int CollisionDetector::CheckPathCollision(const std::vector<Pos>& vec_path, float f_margin_m, bool b_backward, int n_start_idx) const
{
    // exception
    if (vec_path.empty())
        return -1;

    std::shared_lock lock(mtx_);
    std::shared_ptr<const std::vector<int8_t>> map_ptr = map_ptr_;
    Pos o_robot_pos = o_robot_pos_;
    std::shared_lock unlock(mtx_);

    int n_end_idx = (int)vec_path.size();
    float f_ignore_for_charger_dist_m = 0.5f;

    if (vec_path[0].GetMissionType() == Pos::MISSION_NODE_TYPE::CHARGER) {
        int n_idx = 1;
        for (; n_idx < (int)vec_path.size(); ++n_idx) {
            if (vec_path[n_idx].GetZm() - vec_path[0].GetZm() > f_ignore_for_charger_dist_m)
                break;
        }
        if (n_start_idx < n_idx)
            n_start_idx = n_idx;
    }
    if (vec_path.back().GetMissionType() == Pos::MISSION_NODE_TYPE::CHARGER) {
        for (n_end_idx = (int)vec_path.size() - 2; n_end_idx > -1; --n_end_idx) {
            if (vec_path.back().GetZm() - vec_path[n_end_idx].GetZm() > f_ignore_for_charger_dist_m)
                break;
        }
        ++n_end_idx;
    }

    if (map_ptr != nullptr) {
        for (int n_path = n_start_idx; n_path < n_end_idx; ++n_path) {
            int nCenterGridX =
                int(std::floor((vec_path.at(n_path).GetXm() - o_robot_pos.GetXm()) / f_map_resolution_) + 0.5f) + n_map_size_x_px_ / 2;
            int nCenterGridY =
                int(std::floor((vec_path.at(n_path).GetYm() - o_robot_pos.GetYm()) / f_map_resolution_) + 0.5f) + n_map_size_y_px_ / 2;

            if (nCenterGridX < 0 || nCenterGridX >= n_map_size_x_px_ || nCenterGridY < 0 || nCenterGridY >= n_map_size_y_px_) {
                continue;
            }
            int8_t n8_ptr_val = map_ptr->at(nCenterGridX + nCenterGridY * n_map_size_x_px_);
            // 빈공간이면 그냥 지나간다.
            // if ( n8_ptr_val == static_cast<int8_t>(0) ) continue;
            if (n8_ptr_val == static_cast<int8_t>(100))
                return n_path;

            double rad = vec_path.at(n_path).GetRad();
            if (b_backward)
                rad += M_PI;

            float cos_heading = std::cos(rad);
            float sin_heading = std::sin(rad);

            for (float x = f_robot_min_x_m_ - f_margin_m; x < f_robot_max_x_m_ + f_margin_m + f_map_resolution_; x += f_map_resolution_) {
                for (float y = f_robot_min_y_m_ - f_margin_m; y < f_robot_max_y_m_ + f_margin_m + f_map_resolution_;
                     y += f_map_resolution_) {
                    int nGridX = int(std::floor(
                                         (vec_path.at(n_path).GetXm() - o_robot_pos.GetXm() + cos_heading * x - sin_heading * y) /
                                         f_map_resolution_) +
                                     0.5f) +
                        n_map_size_x_px_ / 2;
                    int nGridY = int(std::floor(
                                         (vec_path.at(n_path).GetYm() - o_robot_pos.GetYm() + sin_heading * x + cos_heading * y) /
                                         f_map_resolution_) +
                                     0.5f) +
                        n_map_size_y_px_ / 2;

                    if (nGridX < 0 || nGridX >= n_map_size_x_px_ || nGridY < 0 || nGridY >= n_map_size_y_px_) {
                        continue;
                    }

                    if (map_ptr->at(nGridX + nGridY * n_map_size_x_px_) == 100) {
                        LOG_DEBUG(
                            "[CollisionDetector::CheckPathCollision] ObsPos grid x : %d, grid y : %d, x_m: %.3f, y_m: %.3f", nGridX, nGridY,
                            vec_path.at(n_path).GetXm() - o_robot_pos.GetXm() + cos_heading * x - sin_heading * y,
                            vec_path.at(n_path).GetYm() - o_robot_pos.GetYm() + sin_heading * x + cos_heading * y);
                        LOG_DEBUG(
                            "[CollisionDetector::CheckPathCollision] CollisionIndex: %d, WaypointPos x_m: %.3f, y_m: %.3f, deg: %.3f",
                            n_path, vec_path.at(n_path).GetXm(), vec_path.at(n_path).GetYm(), vec_path.at(n_path).GetDeg());
                        LOG_DEBUG("[CollisionDetector::CheckPathCollision] ObsPosFromWaypoint x_m: %.3f, y_m; %.3f", x, y);
                        return n_path;
                    }
                }
            }
        }
    }

    return -1;
}

bool CollisionDetector::CheckPosCollision(const Pos& o_pos, float f_margin_m, bool b_backward) const
{
    std::shared_lock lock(mtx_);
    std::shared_ptr<const std::vector<int8_t>> map_ptr = map_ptr_;
    Pos o_robot_pos = o_robot_pos_;
    std::shared_lock unlock(mtx_);

    if (map_ptr != nullptr) {
        int nCenterGridX = int(std::floor((o_pos.GetXm() - o_robot_pos.GetXm()) / f_map_resolution_) + 0.5f) + n_map_size_x_px_ / 2;
        int nCenterGridY = int(std::floor((o_pos.GetYm() - o_robot_pos.GetYm()) / f_map_resolution_) + 0.5f) + n_map_size_y_px_ / 2;

        if (nCenterGridX >= 0 && nCenterGridX < n_map_size_x_px_ && nCenterGridY >= 0 && nCenterGridY < n_map_size_y_px_) {
            int8_t n8_ptr_val = map_ptr->at(nCenterGridX + nCenterGridY * n_map_size_x_px_);
            // 빈공간이면 그냥 지나간다.
            // if ( n8_ptr_val == static_cast<int8_t>(0) ) return false;
            if (n8_ptr_val == static_cast<int8_t>(100))
                return true;
        }

        double rad = o_pos.GetRad();
        if (b_backward)
            rad += M_PI;

        float cos_heading = std::cos(rad);
        float sin_heading = std::sin(rad);

        for (float x = f_robot_min_x_m_ - f_margin_m; x < f_robot_max_x_m_ + f_margin_m + f_map_resolution_; x += f_map_resolution_) {
            for (float y = f_robot_min_y_m_ - f_margin_m; y < f_robot_max_y_m_ + f_margin_m + f_map_resolution_; y += f_map_resolution_) {
                int nGridX =
                    int(std::floor((o_pos.GetXm() - o_robot_pos.GetXm() + cos_heading * x - sin_heading * y) / f_map_resolution_) + 0.5f) +
                    n_map_size_x_px_ / 2;
                int nGridY =
                    int(std::floor((o_pos.GetYm() - o_robot_pos.GetYm() + sin_heading * x + cos_heading * y) / f_map_resolution_) + 0.5f) +
                    n_map_size_y_px_ / 2;

                if (nGridX >= 0 && nGridX < n_map_size_x_px_ && nGridY >= 0 && nGridY < n_map_size_y_px_ &&
                    map_ptr->at(nGridX + nGridY * n_map_size_x_px_) == 100) {
                    LOG_DEBUG(
                        "[CollisionDetector::CheckPosCollision] ObsPos grid x : %d, grid y : %d, x_m: %.3f, y_m: %.3f", nGridX, nGridY,
                        o_pos.GetXm() - o_robot_pos.GetXm() + cos_heading * x - sin_heading * y,
                        o_pos.GetYm() - o_robot_pos.GetYm() + sin_heading * x + cos_heading * y);
                    LOG_DEBUG(
                        "[CollisionDetector::CheckPosCollision] Pos x_m: %.3f, y_m: %.3f, deg: %.3f", o_pos.GetXm(), o_pos.GetYm(),
                        o_pos.GetDeg());
                    LOG_DEBUG("[CollisionDetector::CheckPosCollision] ObsPosFromPos x_m: %.3f, y_m; %.3f", x, y);
                    return true;
                }
            }
        }
    }

    return false;
}

bool CollisionDetector::CheckSpinCollision(const Pos& o_pos, float f_margin_m) const
{
    std::shared_lock lock(mtx_);
    std::shared_ptr<const std::vector<int8_t>> map_ptr = map_ptr_;
    Pos o_robot_pos = o_robot_pos_;
    std::shared_lock unlock(mtx_);

    if (map_ptr != nullptr) {
        int nCenterGridX = int(std::floor((o_pos.GetXm() - o_robot_pos.GetXm()) / f_map_resolution_) + 0.5f) + n_map_size_x_px_ / 2;
        int nCenterGridY = int(std::floor((o_pos.GetYm() - o_robot_pos.GetYm()) / f_map_resolution_) + 0.5f) + n_map_size_y_px_ / 2;

        if (nCenterGridX < 0 || nCenterGridX >= n_map_size_x_px_ || nCenterGridY < 0 || nCenterGridY >= n_map_size_y_px_) {
            LOG_WARNING("[CollisionDetector::CheckSpinCollision] o_pos(input data) is out of map");
            return true;
        }
        int8_t n8_ptr_val = map_ptr->at(nCenterGridX + nCenterGridY * n_map_size_x_px_);
        // 빈공간이면 그냥 지나간다.
        // if ( n8_ptr_val == static_cast<int8_t>(0) ) return false;
        if (n8_ptr_val == static_cast<int8_t>(100))
            return true;

        float f_robot_radius_m = f_robot_radius_m_ + f_margin_m;

        for (float x = -f_robot_radius_m; x < f_robot_radius_m + f_map_resolution_; x += f_map_resolution_) {
            for (float y = -f_robot_radius_m; y < f_robot_radius_m + f_map_resolution_; y += f_map_resolution_) {
                if (f_robot_radius_m * f_robot_radius_m < x * x + y * y) {
                    continue;
                }
                int nGridX = int(std::floor((o_pos.GetXm() - o_robot_pos.GetXm() + x) / f_map_resolution_) + 0.5f) + n_map_size_x_px_ / 2;
                int nGridY = int(std::floor((o_pos.GetYm() - o_robot_pos.GetYm() + y) / f_map_resolution_) + 0.5f) + n_map_size_y_px_ / 2;

                if (nGridX < 0 || nGridX >= n_map_size_x_px_ || nGridY < 0 || nGridY >= n_map_size_y_px_) {
                    LOG_DEBUG("[CollisionDetector::CheckSpinCollision] Out of map");
                    LOG_DEBUG(
                        "[CollisionDetector::CheckSpinCollision] ObsPos grid x : %d, grid y : %d, x_m: %.3f, y_m: %.3f", nGridX, nGridY,
                        o_pos.GetXm() - o_robot_pos.GetXm() + x, o_pos.GetYm() - o_robot_pos.GetYm() + y);
                    LOG_DEBUG(
                        "[CollisionDetector::CheckSpinCollision] Pos x_m: %.3f, y_m: %.3f, deg: %.3f", o_pos.GetXm(), o_pos.GetYm(),
                        o_pos.GetDeg());
                    LOG_DEBUG("[CollisionDetector::CheckSpinCollision] ObsPosFromPos x_m: %.3f, y_m; %.3f", x, y);
                    return true;
                }

                if (map_ptr->at(nGridX + nGridY * n_map_size_x_px_) == 100) {
                    LOG_DEBUG(
                        "[CollisionDetector::CheckSpinCollision] ObsPos grid x : %d, grid y : %d, x_m: %.3f, y_m: %.3f", nGridX, nGridY,
                        o_pos.GetXm() - o_robot_pos.GetXm() + x, o_pos.GetYm() - o_robot_pos.GetYm() + y);
                    LOG_DEBUG(
                        "[CollisionDetector::CheckSpinCollision] Pos x_m: %.3f, y_m: %.3f, deg: %.3f", o_pos.GetXm(), o_pos.GetYm(),
                        o_pos.GetDeg());
                    LOG_DEBUG("[CollisionDetector::CheckSpinCollision] ObsPosFromPos x_m: %.3f, y_m; %.3f", x, y);
                    return true;
                }
            }
        }
    }

    return false;
}

int CollisionDetector::CalcCost(const std::vector<Pos>& vec_path, float f_margin_m, bool b_backward, int n_start_idx) const
{
    // exception
    if (vec_path.empty())
        return 0;

    std::shared_lock lock(mtx_);
    std::shared_ptr<const std::vector<int8_t>> map_ptr = map_ptr_;
    Pos o_robot_pos = o_robot_pos_;
    std::shared_lock unlock(mtx_);

    int n_cost = 0;

    if (map_ptr != nullptr) {
        for (int n_path = n_start_idx; n_path < (int)vec_path.size(); ++n_path) {
            int nCenterGridX =
                int(std::floor((vec_path.at(n_path).GetXm() - o_robot_pos.GetXm()) / f_map_resolution_) + 0.5f) + n_map_size_x_px_ / 2;
            int nCenterGridY =
                int(std::floor((vec_path.at(n_path).GetYm() - o_robot_pos.GetYm()) / f_map_resolution_) + 0.5f) + n_map_size_y_px_ / 2;

            if (nCenterGridX < 0 || nCenterGridX >= n_map_size_x_px_ || nCenterGridY < 0 || nCenterGridY >= n_map_size_y_px_) {
                continue;
            }

            int8_t n_max_cost = 0;

            double rad = vec_path.at(n_path).GetRad();
            if (b_backward)
                rad += M_PI;

            float cos_heading = std::cos(rad);
            float sin_heading = std::sin(rad);

            int nGridX, nGridY;

            // fl
            nGridX = int(std::floor(
                             (vec_path.at(n_path).GetXm() - o_robot_pos.GetXm() + cos_heading * (f_robot_max_x_m_ + f_margin_m) -
                              sin_heading * (f_robot_max_y_m_ + f_margin_m)) /
                             f_map_resolution_) +
                         0.5f) +
                n_map_size_x_px_ / 2;
            nGridY = int(std::floor(
                             (vec_path.at(n_path).GetYm() - o_robot_pos.GetYm() + sin_heading * (f_robot_max_x_m_ + f_margin_m) +
                              cos_heading * (f_robot_max_y_m_ + f_margin_m)) /
                             f_map_resolution_) +
                         0.5f) +
                n_map_size_y_px_ / 2;
            if (nGridX >= 0 && nGridX < n_map_size_x_px_ && nGridY >= 0 && nGridY < n_map_size_y_px_ &&
                n_max_cost < map_ptr->at(nGridX + nGridY * n_map_size_x_px_)) {
                n_max_cost = map_ptr->at(nGridX + nGridY * n_map_size_x_px_);
            }
            // fr
            nGridX = int(std::floor(
                             (vec_path.at(n_path).GetXm() - o_robot_pos.GetXm() + cos_heading * (f_robot_max_x_m_ + f_margin_m) -
                              sin_heading * (f_robot_max_y_m_ - f_margin_m)) /
                             f_map_resolution_) +
                         0.5f) +
                n_map_size_x_px_ / 2;
            nGridY = int(std::floor(
                             (vec_path.at(n_path).GetYm() - o_robot_pos.GetYm() + sin_heading * (f_robot_max_x_m_ + f_margin_m) +
                              cos_heading * (f_robot_max_y_m_ - f_margin_m)) /
                             f_map_resolution_) +
                         0.5f) +
                n_map_size_y_px_ / 2;
            if (nGridX >= 0 && nGridX < n_map_size_x_px_ && nGridY >= 0 && nGridY < n_map_size_y_px_ &&
                n_max_cost < map_ptr->at(nGridX + nGridY * n_map_size_x_px_)) {
                n_max_cost = map_ptr->at(nGridX + nGridY * n_map_size_x_px_);
            }
            // ml
            nGridX = int(std::floor(
                             (vec_path.at(n_path).GetXm() - o_robot_pos.GetXm() - sin_heading * (f_robot_max_y_m_ + f_margin_m)) /
                             f_map_resolution_) +
                         0.5f) +
                n_map_size_x_px_ / 2;
            nGridY = int(std::floor(
                             (vec_path.at(n_path).GetYm() - o_robot_pos.GetYm() + cos_heading * (f_robot_max_y_m_ + f_margin_m)) /
                             f_map_resolution_) +
                         0.5f) +
                n_map_size_y_px_ / 2;
            if (nGridX >= 0 && nGridX < n_map_size_x_px_ && nGridY >= 0 && nGridY < n_map_size_y_px_ &&
                n_max_cost < map_ptr->at(nGridX + nGridY * n_map_size_x_px_)) {
                n_max_cost = map_ptr->at(nGridX + nGridY * n_map_size_x_px_);
            }
            // mr
            nGridX = int(std::floor(
                             (vec_path.at(n_path).GetXm() - o_robot_pos.GetXm() - sin_heading * (f_robot_max_y_m_ - f_margin_m)) /
                             f_map_resolution_) +
                         0.5f) +
                n_map_size_x_px_ / 2;
            nGridY = int(std::floor(
                             (vec_path.at(n_path).GetYm() - o_robot_pos.GetYm() + cos_heading * (f_robot_max_y_m_ - f_margin_m)) /
                             f_map_resolution_) +
                         0.5f) +
                n_map_size_y_px_ / 2;
            if (nGridX >= 0 && nGridX < n_map_size_x_px_ && nGridY >= 0 && nGridY < n_map_size_y_px_ &&
                n_max_cost < map_ptr->at(nGridX + nGridY * n_map_size_x_px_)) {
                n_max_cost = map_ptr->at(nGridX + nGridY * n_map_size_x_px_);
            }

            n_cost += static_cast<int>(n_max_cost);
        }
    }

    return n_cost;
}

}  // namespace NaviFra
