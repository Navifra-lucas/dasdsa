#pragma once

#include "common/pch.h"
#include "common/scan2d.h"

#include <Poco/Dynamic/Var.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>

#define MAX_LIDAR_NUM 3600

namespace ANSWER {
struct lidar_merger_2d_parameters {
    float min_distance = 0.05;
    float max_distance = 60.0;
    float x_front_min_distance = 0.5;
    float x_rear_min_distance = 0.5;
    float y_left_min_distance = 0.5;
    float y_right_min_distance = 0.5;
    float merge_distance_threshold = 0.05;  // Threshold for merging points

    Poco::JSON::Object::Ptr min_distance_;
    // float min_distance = 0.05;
    // float max_distance = 60.0;
    // float x_front_min_distance = 0.5;
    // float x_rear_min_distance = 0.5;
    // float y_left_min_distance = 0.5;
    // float y_right_min_distance = 0.5;
    // float merge_distance_threshold = 0.05;  // Threshold for merging points
};

class LiDARMerger2D {
private:
    bool CheckBounds(const float &x, const float &y);

    lidar_merger_2d_parameters parameters_;
    std::vector<Scan2D> scans_for_ui_;

    int GetAngleIndex(const Point2D &point);

public:
    using Ptr = std::shared_ptr<LiDARMerger2D>;
    LiDARMerger2D();
    ~LiDARMerger2D();

    const Scan2D MergeLiDARs(std::vector<Scan2D> &scans);
    void UpdateParameters();
};
}  // namespace ANSWER