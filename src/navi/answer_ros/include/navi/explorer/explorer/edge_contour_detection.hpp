/*
 * @file	: edge_contour_detection.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_EDGE_CONTOUR_DETECTION_HPP_
#define EXPLORER_EDGE_CONTOUR_DETECTION_HPP_

#include <vector>

#include <opencv2/opencv.hpp>

namespace NVFR {

class EdgeContourDetector
{
public:
  EdgeContourDetector();
  ~EdgeContourDetector() = default;

  std::vector< std::vector<cv::Point> > DetectContours(const cv::Mat& grayImage, bool remove_noise=true);

  std::vector<cv::Point> GetCentroid(const std::vector< std::vector<cv::Point> >& contours);

private:
  std::vector< std::vector<cv::Point> > IgnoreSmallContours(
    const std::vector< std::vector<cv::Point> >& contours,
    double min_length);
  cv::Point computeCenter(const std::vector<cv::Point>& points);

  const double LowerCannyBound;
  const double UpperCannyBound;

};

} // namespace NVFR

#endif
