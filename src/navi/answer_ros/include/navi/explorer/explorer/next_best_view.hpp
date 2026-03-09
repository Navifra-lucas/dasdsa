/*
 * @file	: next_best_view.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_NEXT_BEST_VIEW_HPP_
#define EXPLORER_NEXT_BEST_VIEW_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <queue>

#include <opencv2/opencv.hpp>

#include "explorer/param/sub/expr_weight_param.hpp"
#include "explorer/utils.hpp"

namespace NVFR {

class ViewSample {
public:
  ViewSample():pt(0,0),length_cost(0.f) {}
  ViewSample(int _x, int _y):pt(_x, _y),length_cost(0.f) {}
  ViewSample(const cv::Point& vsp):pt(vsp),length_cost(0.f) {}
  ViewSample(int _x, int _y, float _length_cost):pt(_x, _y),length_cost(_length_cost) {}
  ViewSample(const cv::Point& vsp, float _length_cost):pt(vsp),length_cost(_length_cost) {}
  ViewSample(const EUS::CV_Node& node):pt(node.pt),length_cost(0.f) {}
  ViewSample(const EUS::CV_Node* node):pt(node->pt),length_cost(0.f) {}
  ViewSample(const EUS::CV_Node& node, float _length_cost):pt(node.pt),length_cost(_length_cost) {}
  ViewSample(const EUS::CV_Node* node, float _length_cost):pt(node->pt),length_cost(_length_cost) {}

  cv::Point pt;
  float length_cost;
  float explore_cost=0.0f;

  float calcCost() const;

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const ViewSample& node);
};

struct CmpVS {
  bool operator() (const ViewSample& vs1, const ViewSample& vs2) const;
};

class NextBestView
{
public:
  NextBestView();
  ~NextBestView();

  size_t getSize() const { return pqNextViewSamples_.size(); }
  bool empty() const { return pqNextViewSamples_.empty(); }

  ViewSample popNextBestView();

  void clear();

  size_t execute(const cv::Mat& mapImage,
    const std::vector<cv::Point>& nextViewPoints,
    const cv::Point& robotPos,
    const EUS::SensorInfo& sensorInfo,
    float mapResM, float padding, float minUnknownLength,
    const ExprWeightParam_t& st_expr_w);

private:
  std::priority_queue<ViewSample, std::vector<ViewSample>, CmpVS> pqNextViewSamples_;

  std::vector<ViewSample> calcPathLength(const cv::Mat& mapImage,
    const std::vector<cv::Point>& nextViewPoints,
    const cv::Point& robotPos,
    float mapResM, float padding);

  float calcUnknownRay(const cv::Mat& mapImage, const cv::Point& thisPoint,
    const EUS::SensorInfo& sensorInfo, float mapResM);

};

} // namespace NVFR

#endif
