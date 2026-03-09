/*
 * @file	: data_manager.hpp
 * @date	: Oct 14, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: set & get data | monitoring data 
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef DATA_MANAGER_HPP_
#define DATA_MANAGER_HPP_

#include <mutex>
#include <shared_mutex>

#include "utils/time_util.hpp"

namespace NVFR {

template <typename T>
class DataManager
{
public:
  DataManager()
  : n_max_delay_msec_(static_cast<int64_t>(500))
  , tp_get_(TimeUtil::GetNow())
  {}
  DataManager(const T& data)
  : n_max_delay_msec_(static_cast<int64_t>(500))
  , tp_get_(TimeUtil::GetNow())
  , data_(data)
  {}
  virtual ~DataManager()
  {}

  void SetMaxMilliSec(int n_max_delay_msec)
  {
    n_max_delay_msec_ = n_max_delay_msec;
  }

  bool IsOver() const
  {
    std::shared_lock<std::shared_mutex> lk(mtx_);
    bool b_over =
      TimeUtil::GetMilliSec(tp_get_) > n_max_delay_msec_
      ? true
      : false;
    return b_over;
  }

  const T& Get() const
  {
    std::shared_lock<std::shared_mutex> ulk(mtx_);
    return data_;
  }

  void Set(const T& data)
  {
    std::unique_lock<std::shared_mutex> lk(mtx_);
    tp_get_ = TimeUtil::GetNow();
    data_ = data;
  }

  void Move(T& data)
  {
    std::unique_lock<std::shared_mutex> lk(mtx_);
    tp_get_ = TimeUtil::GetNow();
    data_ = std::move(data);
  }

private:
  int64_t n_max_delay_msec_;
  TimeUtil::_tp tp_get_;
  T data_;
  mutable std::shared_mutex mtx_;

};

} // namespace NVFR

#endif
