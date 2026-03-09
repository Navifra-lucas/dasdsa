/**
 * @class global map builder 2d
 * @brief  build global map(occupancy grid map) by stitching localmaps.
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef GLOBAL_MAP_BUILDER_2D_H
#define GLOBAL_MAP_BUILDER_2D_H

#include "2d_slam/common/slam_parameter_container.h"
#include "2d_slam/globalmapper/mapper.h"
#include "2d_slam/localmapper/localmap/localmap2d.h"
#include "common/lookup_table.h"
#include "common/pose2d.h"
#include "glog/logging.h"

#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "Eigen/Core"

using namespace std;

namespace ANSWER {
namespace SLAM2D {

class GlobalMapBuilder2D {
private:
    /* data */
    thread global_map_builder_thread_;
    mutex auto_save_flag_mutex_;
    bool b_global_map_build_ = false;

    void BuildGlobalMap();
    void UpdateGlobalMapSize();
    bool CheckMapSize(const Vector2f& point);
    // vector<int8_t> global_map_;
    vector<int16_t> global_map_;
    string map_frame_;
    string map_topic_;
    string map_path_;
    string map_name_;
    bool b_auto_save_ = true;
    int last_index_ = -1;
    const slam_parameter_container* slam_param_;
    Pose2D prev_odom_pose_;
    Pose2D delta_odom_pose_;
    shared_ptr<Localmap2D> firtst_map_;
    deque<Scan2D> scan_vec_;
    std::function<void(const std::vector<int16_t>&)> update_map_func_;

public:
    GlobalMapBuilder2D(/* args */);
    ~GlobalMapBuilder2D();

    void Initialize(string map_frame, string map_topic_, string map_path, string map_name_);
    bool SaveMap();
    void StopAutoSave()
    {
        if (auto_save_flag_mutex_.try_lock()) {
            b_auto_save_ = false;
            auto_save_flag_mutex_.unlock();
        }
    }
    void SetParam(const slam_parameter_container* slam_param) { slam_param_ = slam_param; }

    void DrawFirstScan(const Scan2D first_scan);
    static GlobalMapBuilder2D* GetInstance()
    {
        static GlobalMapBuilder2D s;
        return &s;
    }
    const vector<int16_t>& GetGlobalMap() { return global_map_; }
    // void SetUpdateMapFunc(std::function<void(const std::vector<int16_t>&)> update_map_func) { update_map_func_ = update_map_func; }
};

}  // namespace SLAM2D
}  // namespace ANSWER
#endif