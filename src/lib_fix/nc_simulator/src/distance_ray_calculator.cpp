#include "distance_ray_calculator.hpp"

#include "core/util/logger.hpp"

using namespace std;
namespace NaviFra {
DistanchRayCalculator::DistanchRayCalculator()
{
}

void DistanchRayCalculator::Initialize(const core_msgs::MapDB::ConstPtr& msg, float f_max_search_range_m)
{
    b_start_make_map_ = true;
    f_max_search_range_m_ = f_max_search_range_m;

    f_map_min_x_ = msg->map_min_x;
    f_map_min_y_ = msg->map_min_y;
    f_map_max_x_ = msg->map_max_x;
    f_map_max_y_ = msg->map_max_y;

    if (abs(f_map_max_x_ - f_map_min_x_) > 200 || abs(f_map_max_y_ - f_map_min_y_) > 200) {
        f_resolution_ = 0.04;
    }
    else if (abs(f_map_max_x_ - f_map_min_x_) > 60 || abs(f_map_max_y_ - f_map_min_y_) > 60) {
        f_resolution_ = 0.02;
    }
    else {
        f_resolution_ = 0.01;
    }
    DistanceMap(msg);
    b_start_make_map_ = false;
}

float DistanchRayCalculator::CalcPointLength(int n_map_x_px, int n_map_y_px, float f_heading_angle_rad)
{
    if (b_start_make_map_)
        return 0;

    float ray_direction_x = cosf(f_heading_angle_rad);
    float ray_direction_y = sinf(f_heading_angle_rad);
    int search_x = 0;
    int search_y = 0;
    float t = 0.0;
    float f_res = f_resolution_;
    while (t < f_max_search_range_m_ / f_res) {
        search_x = n_map_x_px + ray_direction_x * t;
        search_y = n_map_y_px + ray_direction_y * t;
        if (search_x > (f_map_max_x_ - f_map_min_x_) / f_res || search_x < 0 || search_y > (f_map_max_y_ - f_map_min_y_) / f_res ||
            search_y < 0) {
            return 0;
        }
        float d = vec_distance_map_[search_x][search_y];

        if (d <= f_dist_threshold_) {
            float f_min_d = 1000;
            float xd = search_x - n_map_x_px;
            float yd = search_y - n_map_y_px;

            for (int i = -10; i <= 10; i++) {
                search_x = n_map_x_px + ray_direction_x * (t + i);
                search_y = n_map_y_px + ray_direction_y * (t + i);
                if (search_x >= (f_map_max_x_ - f_map_min_x_) / f_res || search_x <= 0 ||
                    search_y >= (f_map_max_y_ - f_map_min_y_) / f_res || search_y <= 0)
                    continue;

                d = vec_distance_map_[search_x][search_y];
                if (d < f_min_d) {
                    f_min_d = d;
                    xd = search_x - n_map_x_px;
                    yd = search_y - n_map_y_px;
                }
                if (d <= 0)
                    break;
            }

            return hypot(xd, yd) * f_resolution_;
        }
        t += std::max<float>(d * f_step_coeff_, 1.0);
    }
    return 0;
}

void DistanchRayCalculator::DistanceMap(const core_msgs::MapDB::ConstPtr& msg)
{
    const float MAX_VALUE = std::numeric_limits<float>::max();

    long unsigned int width = int((msg->map_max_x - msg->map_min_x) / f_resolution_) + 1;
    long unsigned int height = int((msg->map_max_y - msg->map_min_y) / f_resolution_) + 1;

    // NLOG(info) << "1width " << width << " height " << height;
    if (width > 1000000 || height > 1000000) {
        NLOG(info) << "Too High width height!";
        return;
    }
    std::vector<std::size_t> grid_size({width, height});
    dt::MMArray<float, 2> f(grid_size.data());
    dt::MMArray<std::size_t, 2> indices(grid_size.data());
#pragma omp parallel for
    for (int y = 0; y < height - 1; y++) {
        for (int x = 0; x < width - 1; x++) {
            f[x][y] = MAX_VALUE;
        }
    }
    // NLOG(info) << "2width " << width << " height " << height;

#pragma omp parallel for
    for (int i = 0; i < msg->map_db.poses.size(); i++) {
        int x = int((msg->map_db.poses[i].position.x - msg->map_min_x) / f_resolution_);  // x
        int y = int((msg->map_db.poses[i].position.y - msg->map_min_y) / f_resolution_);  // y
        if (x < width - 1 && y < height - 1)
            f[x][y] = 0.0f;
    }
    // NLOG(info) << "3width " << width << " height " << height;
    vec_distance_map_.resize(width, std::vector<float>(height, 1));

    dt::DistanceMap::distanceTransformL2(f, f, indices, false);
    NLOG(info) << "vec_distance_map_ " << vec_distance_map_.size();
    for (int y = 0; y < height - 1; y++) {
        for (int x = 0; x < width - 1; x++) {
            vec_distance_map_[x][y] = f[x][y];
        }
    }
    // NLOG(info) << "4width " << width << " height " << height;
}
}  // namespace NaviFra
