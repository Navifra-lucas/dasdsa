/*
 * @file	: slam_nodes.hpp
 * @date	: July 14, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: consider graph nodes of slam
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_SLAM_NODES_HPP_
#define EXPLORER_SLAM_NODES_HPP_

#include <vector>

#include <opencv2/opencv.hpp>

#include "utils/pose.hpp"

namespace NVFR {

class SlamNodes
{
public:
  SlamNodes();
  ~SlamNodes() = default;

  std::vector<cv::Point> RemoveVeiwSamples(
    const std::vector<cv::Point>& viewSamples,
    const Polygon& slamNodes, float minLength,
    const cv::Mat& image, float offsetX, float offsetY, float res);

private:

};

} // namespace NVFR

#endif
