#ifndef __NC_NAVI_RANSAC__
#define __NC_NAVI_RANSAC__

#include <omp.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <boost/any.hpp> 

using ClusterIndices = std::vector<std::vector<int>>;

namespace navi_ransac {

struct RansacParam {
    bool b_use = true;                // RANSAC 평면 모델 추출 사용 여부
    int n_k_search = 50;              // 사용 포인트 개수
    int u_iteration = 3;              // 반복 횟수

    double d_distance_weight = 0.1;   // 미사용
    double d_prob = 0.99;             // 미사용

    double d_distance_m = 0.03;                // 평면 감지 inlier 구별 거리
    double d_distance_m_default = 0.1;         // 평소 inlier 구별 거리
    double d_hole_detect_distance_m = 0.1;     // 그레이트 감지 시 거리

    double d_voxel_leaf_size_m = 0.025;        // 다운샘플링 leaf size

    double d_x_min_th_m = -6.0;                // 감지 범위 X 최소
    double d_x_max_th_m = 6.0;                 // 감지 범위 X 최대
    double d_y_min_th_m = -7.0;                // 감지 범위 Y 최소
    double d_y_max_th_m = 7.0;                 // 감지 범위 Y 최대
    double d_z_min_th_m = -0.1;                // 감지 범위 Z 최소
    double d_z_max_th_m = 2.0;                 // 감지 범위 Z 최대

    double d_x_filter_m = 0.48;                // 앞뒤 방향 포인트 필터링 범위
    double d_y_filter_m = 0.33;                // 좌우 방향 필터링
    double d_z_filter_m = 0.30;                // 상하 방향 필터링

    double d_collision_ratio_x = 0.9;
    double d_collision_ratio_y = 0.9;

    double d_ground_limit_m = 0.12;            // 평면 감지 포인트 임계 높이
    double d_ground_limit_m_default = 0.25;    // 평상시 평면 높이
    double d_hole_detect_ground_limit_m = 0.1; // 그레이트 감지 시 평면 높이

};

struct Parameters
{
    /* obstacle detector parameters */
    bool b_is_sim = false;
    bool b_t_mini = false;
    bool b_vanjee = false;
    bool b_use_external_wrapper = false;
    // int n_severity_lv = severity_level::info;
    bool b_stand_alone = false;
    bool b_use_dynamic_roi;
    bool b_do_post_processing = true;
    double d_z_upper_max_m;
    float f_docking_x_filter_m;
    int n_time_update_hz;

    /* ransac parameters */
    RansacParam ransac_param;
};

class RANSAC
{
private:
    std::shared_ptr<Parameters> st_parameters_;
    std::map<std::string, std::function<void(const boost::any &)> > map_callback_pt_;

    ros::NodeHandle nh_;
    ros::Subscriber hole_state_sub_;

public:
    // RANSAC() = default;
    RANSAC();
	virtual ~RANSAC();

    bool SetParam(std::shared_ptr<Parameters> st_parameters);

    bool RegistCallbackFunc(const std::string &str_cbf_name, const std::function<void(const boost::any &)> &pt_func);
    std::vector<pPointCloudXYZ> RansacProcess(const pPointCloudXYZ& input_cloud_ptr);

    void RegistListener();


private:
    // Common function 
    bool Notify(const std::string &str_cbf_name, const boost::any &any_type_var);

    // RANSAC function
    bool PassThrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr, const pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud_ptr);
    bool VoxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr, const pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud_ptr);
    std::vector<pPointCloudXYZ> FilterGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr);
};

} // namespace RANSAC

#endif // __NC_NAVI_RANSAC__