/*
 * @file	: lane_info.hpp
 * @date	: Dec 12, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for lane information
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef LANE_INFO_HPP_
#define LANE_INFO_HPP_

#include <iostream>
#include <memory>

namespace NVFR {

class LaneInfo
{
public:
  using Ptr = std::shared_ptr<LaneInfo>;
  using ConstPtr = std::shared_ptr<const LaneInfo>;

  LaneInfo();
  LaneInfo(int n_left_num, int n_right_num, double d_width);
  virtual ~LaneInfo() {};

  int GetLeftNum() const { return n_left_num_; };
  int GetRightNum() const { return n_right_num_; };
  double GetWidth() const { return d_width_; };

  void SetLeftNum(int n_left_num);
  void SetRightNum(int n_right_num);
  void SetWidth(double d_width);
  void SetLaneInfo(int n_left_num, int n_right_num, double d_width);

  LaneInfo Reverse() const;

  /**
   * @brief check this lane info is valid or not
   * @return true (valid) / false (invalid)
  */
  bool IsValid() const;

  bool operator==(const LaneInfo& rhs) const;
  bool operator!=(const LaneInfo& rhs) const;

  virtual std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const LaneInfo& o);

private:
  int n_left_num_;
  int n_right_num_;
  double d_width_;
};

} // namespace NVFR

#endif
