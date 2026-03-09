/*
 * @file	: view_point_sampling.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_VIEW_POINT_SAMPLING_HPP_
#define EXPLORER_VIEW_POINT_SAMPLING_HPP_

#include <vector>

#include <opencv2/opencv.hpp>

#include "explorer/utils.hpp"

namespace NVFR {

class ViewPointSampling
{
public:
  ViewPointSampling();
  ~ViewPointSampling() = default;

  std::vector<cv::Point> DetectViewPointSamples(
    const cv::Mat& mapImage, const cv::Mat& voronoiImage,
    const std::vector<cv::Point>& frontiers,
    float minLength, float maxLength,
    const EUS::SensorInfo& sensorInfo, float mapResM);

private:
  cv::Point2f calcNormalVector(const cv::Mat& mapImage,
    cv::Point thisPoint, float radis, float mapResM);

};

} // namespace NVFR

#endif
