/*
 * @file	: voronoi_detection.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_VORONOI_DETECTION_HPP_
#define EXPLORER_VORONOI_DETECTION_HPP_

#include <vector>

#include <opencv2/opencv.hpp>

namespace NVFR {

class VoronoiDetector
{
public:
  VoronoiDetector();
  ~VoronoiDetector() = default;

  cv::Mat GetVoronoiEdges(const cv::Mat& grayImage, float min_dist);

private:

};

} // namespace NVFR

#endif
