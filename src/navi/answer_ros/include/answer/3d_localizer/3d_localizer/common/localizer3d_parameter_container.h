#pragma once
namespace ANSWER {

class Localizer3DParameterContainer {
private:
public:
    Localizer3DParameterContainer(/* args */)
        : max_range(50.f)
        , min_range(0.1f)
        , voxel_size(0.1f)
        , max_matching_dist(0.5f)
        , estimation_cond_dist_m(0.5f)
        , estimation_cond_cond_deg(5.f)
        , use_voxel_filter(false)
    {
    }
    ~Localizer3DParameterContainer() {}

    float max_range;
    float min_range;
    float voxel_size;
    float max_matching_dist;
    float estimation_cond_dist_m;
    float estimation_cond_cond_deg;
    bool use_voxel_filter;
};

}  // namespace ANSWER