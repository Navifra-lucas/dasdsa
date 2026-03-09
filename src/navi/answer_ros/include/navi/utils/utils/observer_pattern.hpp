/*
 * @file	: observer_pattern.hpp
 * @date	: Dec 12, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for drive information
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef OBSERVER_PATTERN_HPP_
#define OBSERVER_PATTERN_HPP_

#include <memory>
#include <vector>

#include "logger/logger.h"

namespace NVFR {

template <typename... Args>
class Observer
{
public:
  ~Observer() = default;
  virtual void Update(Args...) = 0;

};

template <typename... Args>
class Subject
{
  using ObserverPtr = std::shared_ptr< Observer<Args...> >;
public:
  ~Subject() = default;
  bool Regist(ObserverPtr observer_ptr)
  {
    for (const auto& o_observer_ptr : vec_observer_ptr_)
    {
      if (o_observer_ptr == observer_ptr) {
        LOG_WARN("[Subject::Regist] {:p} is registed already", static_cast<const void*>(observer_ptr.get()));
        return false;
      }
    }
    vec_observer_ptr_.emplace_back(observer_ptr);
    LOG_INFO("[Subject::Regist] regist {:p}", static_cast<const void*>(observer_ptr.get()));
    return true;
  }
  bool IsRegisted(ObserverPtr observer_ptr) const
  {
    for (size_t i=0; i < vec_observer_ptr_.size(); ++i)
    {
      if (vec_observer_ptr_[i] == observer_ptr) {
        LOG_DEBUG("[Subject::IsRegisted] it is registed, {:p}", static_cast<const void*>(observer_ptr.get()));
        return true;
      }
    }
    LOG_WARN("[Subject::IsRegisted] it is not registed, {:p}", static_cast<const void*>(observer_ptr.get()));
    return false;
  }
  bool Remove(ObserverPtr observer_ptr)
  {
    for (size_t i=0; i < vec_observer_ptr_.size(); ++i)
    {
      if (vec_observer_ptr_[i] == observer_ptr) {
        vec_observer_ptr_.erase(vec_observer_ptr_.begin() + i);
        LOG_INFO("[Subject::Remove] remove {:p}", static_cast<const void*>(observer_ptr.get()));
        return true;
      }
    }
    LOG_WARN("[Subject::Remove] {:p} is not registed yet", static_cast<const void*>(observer_ptr.get()));
    return false;
  }

protected:
  std::vector<ObserverPtr> vec_observer_ptr_;

  virtual void Notify() = 0;

};

} // namespace NVFR

#endif
