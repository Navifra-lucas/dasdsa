/*
 * @file	: pcl_cluster.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: point cloud clustering based on statistical method
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef PCL_CLUSTER_HPP_
#define PCL_CLUSTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "utils/param/sub/pcl_cluster_param.hpp"

namespace NVFR {

class PclCluster
{
public:
  enum Method : int
  {
    RAW = 0,
    RADIUS = 1,
    STATISTICSAL = 2,
  };
  PclCluster() {};
  virtual ~PclCluster() = default;

  void SetParam(const PclClusterParam_t& st_param);

  pcl::PointCloud<pcl::PointXYZ>::Ptr RemoveOutlier(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
  PclClusterParam_t st_param_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr ROR(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr SOR(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

};

} // namespace NVFR

#endif
