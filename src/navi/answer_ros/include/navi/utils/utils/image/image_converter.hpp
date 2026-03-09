/*
 * @file	: image_converter.hpp
 * @date	: jun 9, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: image converter
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef IMAGE_CONVERTOR_HPP_
#define IMAGE_CONVERTOR_HPP_

#include <tuple>

#include <opencv2/opencv.hpp>

#include "utils/grid_map/map_info.hpp"
#include "utils/pose.hpp"

namespace NVFR {

namespace ImageConverter {

/**
 * @brief convert image to grid map
 * @param grayImage cv::Mat image made of gray scale
 * @param offsetX offset x [m]
 * @param offsetY offset y [m]
 * @param res resolution [m]
 * @return std::tuple< bool, MapInfo_t, std::vector<int8_t> > : (success, map_info, cost_map)
*/
std::tuple< bool, MapInfo_t, std::vector<int8_t> > imageToMap(
  const cv::Mat& grayImage, float offsetX, float offsetY, float res);

std::tuple< bool, cv::Mat > mapToImage(
  const MapInfo_t st_map_info, const std::vector<int8_t> grid_map);

Pos pointToPos(
  const cv::Point& pt, const cv::Mat& grayImage, float offsetX, float offsetY, float res);

cv::Point posToPoint(
  const Pos& o_pos, const cv::Mat& grayImage, float offsetX, float offsetY, float res);

Pose pointToPose(
  const cv::Point& pt, const cv::Mat& grayImage, float offsetX, float offsetY, float res);

cv::Point poseToPoint(
  const Pose& o_pose, const cv::Mat& grayImage, float offsetX, float offsetY, float res);

Polygon pointsToPolygon(const std::vector<cv::Point>& points,
  const cv::Mat& grayImage, float offsetX, float offsetY, float res);

std::vector<cv::Point> polygonToPoints(const Polygon& polygon,
  const cv::Mat& grayImage, float offsetX, float offsetY, float res);

Path pointsToPath(const std::vector<cv::Point>& points,
  const cv::Mat& grayImage, float offsetX, float offsetY, float res);

std::vector<cv::Point> pathToPoints(const Path& path,
  const cv::Mat& grayImage, float offsetX, float offsetY, float res);

} // namespace ImageConverter
namespace ICVT = ImageConverter;
} // namespace NVFR

#endif
