/*
 * @file	: utils.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_UTILS_HPP_
#define EXPLORER_UTILS_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <random>

#include <opencv2/opencv.hpp>

#include "utils/common_math.hpp"
#include "utils/pose.hpp"

namespace NVFR {

namespace ExplorerUtils {

// Sensor information
class SensorInfo {
public:
  float getMaxRayRangeM() const { return maxRayRangeM; }
  float getStartAngleDeg() const { return startAngleDeg; }
  float getAngleRangeDeg() const { return angleRangeDeg; }
  float getAngleResDeg() const { return angleResDeg; }
  float getStartAngleRad() const { return startAngleDeg * CM::Deg2Rad; }
  float getAngleRangeRad() const { return angleRangeDeg * CM::Deg2Rad; }
  float getAngleResRad() const { return angleResDeg * CM::Deg2Rad; }
  int getAngleCount() const { return angleRangeDeg / angleResDeg; }

  void setMaxRayRangeM(float val) { maxRayRangeM = val; }
  void setStartAngleDeg(float val) { startAngleDeg = val; }
  void setAngleRangeDeg(float val) { angleRangeDeg = val; }
  void setAngleResDeg(float val) { angleResDeg = val; }
  void setStartAngleRad(float val) { startAngleDeg = val * CM::Rad2Deg; }
  void setAngleRangeRad(float val) { angleRangeDeg = val * CM::Rad2Deg; }
  void setAngleResRad(float val) { angleResDeg = val * CM::Rad2Deg; }

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const SensorInfo& o);

private:
  float maxRayRangeM = 10.0;
  float startAngleDeg = 0.0;
  float angleRangeDeg = 360.0;
  float angleResDeg = 1.0;
};

// CV calculator utils
inline int calcLengthSq(const cv::Point& pt0, const cv::Point& pt1)
{
  return (pt1.x - pt0.x) * (pt1.x - pt0.x) + (pt1.y - pt0.y) * (pt1.y - pt0.y);
}

inline float calcLength(const cv::Point& pt0, const cv::Point& pt1)
{
  return std::sqrt(static_cast<float>(calcLengthSq(pt0, pt1)));
}

inline bool outOfMap(const cv::Mat& mapImage, const cv::Point& pt)
{
  if (pt.x < 0 || pt.y < 0 || pt.x >= mapImage.cols || pt.y >= mapImage.rows) {
    return true;
  }
  return false;
}

cv::Mat paddingMap(const cv::Mat& image,
  float paddingSizeM, float mapResM, uchar padding_val=0);

cv::Mat removeGrayNoise(const cv::Mat& image);

// Draw image
cv::Mat DrawPoints(const cv::Mat& image,
  const std::vector<cv::Point>& points, bool b_color=false);

cv::Mat DrawLines(const cv::Mat& image,
  const std::vector<cv::Point>& points, int lineSize);
cv::Mat DrawLines(const cv::Mat& image, const Polygon& polygon,
  float radius, float offsetX, float offsetY, float res);
cv::Mat DrawLines(const cv::Mat& image, const Path& path,
  float radius, float offsetX, float offsetY, float res);

cv::Mat DrawContours(const cv::Mat& image,
  const std::vector< std::vector<cv::Point> >& contours, bool b_color=false);

cv::Mat DrawFeatureImage(const cv::Mat& image, const cv::Mat& featureImage);

// planner

/**
 * @note type: unsigned char
 * @param NONE, VISIT, CLOSE
*/
enum class STATUS : unsigned char {
  NONE = 0,
  VISIT = 1,
  CLOSE,
};


/**
 * @brief Node (cv::Point) for dijkstra
 * @param x pos x in image map
 * @param y pos y in image map
 * @param cost cost (distance)
 */
class CV_Node
{
public:
  CV_Node():
    pt(0,0),cost(0.f) {}
  CV_Node(int _x, int _y):
    pt(_x,_y),cost(0.f) {}
  CV_Node(const cv::Point& _pt):
    pt(_pt),cost(0.f) {}
  CV_Node(int _x, int _y, float _length_cost):
    pt(_x,_y),cost(_length_cost) {}
  CV_Node(const cv::Point& _pt, float _length_cost):
    pt(_pt),cost(_length_cost) {}
  virtual ~CV_Node() = default;

  cv::Point pt;

  float cost;

  bool b_view_sample = false;
  STATUS e_status = STATUS::NONE;

  void SetPoint(int _x, int _y);

  CV_Node operator+(const CV_Node& rhs) const;
  CV_Node operator-(const CV_Node& rhs) const;
  bool operator==(const CV_Node& rhs) const; // compare grid pos
  bool operator!=(const CV_Node& rhs) const; // compare grid pos

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const CV_Node& node);
};

struct CmpCV_Node {
  bool operator() (const CV_Node* cvNode1, const CV_Node* cvNode2) const;
};

} // namespace ExplorerUtils
namespace EUS = ExplorerUtils;
} // namespace NVFR

#endif
