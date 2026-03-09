#pragma once

#include "2d_slam/common/data_base.h"
#include "2d_slam/globalmapper/mapper.h"
#include "2d_slam/localmapper/localmap/localmap2d.h"
#include "Poco/FileStream.h"
#include "Poco/JSON/Object.h"
#include "common/ceres_util/normalize_angle.h"
#include "common/lookup_table.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "common/time_checker.h"
#include "common/voxel_filter.h"
#include "logger/logger.h"
#include "opencv2/core.hpp"

#include <signal.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/tbb.h>

#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

using namespace cv;

namespace ANSWER {
namespace SLAM2D {

class MapGenerator {
private:
    /* data */
    float resolution_;
    float out_occ_thresh_;
    float out_free_thresh_;
    int n_scan_data_usage_interval_;
    std::string s_write_map_path_;
    std::string s_write_map_name_;
    std::vector<int16_t> global_map_;
    bool DoRayTracing(
        const Mat global_map, const Pose2D map_to_observation, const Pose2D map_to_ptr, int& index_x, int& index_y,
        cv::Mat& filtered_image);

    void Initialize(const slam_parameter_container* slam_params);

    std::unique_ptr<Mapper> mapper_;
    std::unique_ptr<DataBase> db_;
    std::unique_ptr<Localmap2D> resolution_global_map_;
    bool DrawGridMap();

    void ReadFile(const string& map_path, const string& map_name, Mapper* mapper, DataBase* db);
    void SaveFinalMapJson(const cv::Mat& image, Mapper* mapper);

    std::thread map_generator_thread_;

public:
    MapGenerator(const slam_parameter_container* slam_params);
    ~MapGenerator();

    void StartDrawMap();
};

}  // namespace SLAM2D
}  // namespace ANSWER