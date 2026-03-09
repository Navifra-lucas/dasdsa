/*
 * @file	: notify_cb_func.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: utils of notify callback functions
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_NOTIFY_CB_FUNC_HPP_
#define NAVIFRA_NOTIFY_CB_FUNC_HPP_

#include <string>
#include <vector>
#include <map>
#include <functional>

#include "logger/logger.h"

namespace NVFR {

template <typename T>
class NotifyCbFunc
{
public:
  ~NotifyCbFunc() = default;

  bool RegistCbFunc(const std::string& s_name, std::function<bool(T)> func)
  {
    if (map_func_.find(s_name) == map_func_.end()) {
      map_func_[s_name] = func;
      LOG_INFO("Regist func <{}>", s_name.c_str());
      return true;
    }
    LOG_DEBUG("Already func is registed <{}>", s_name.c_str());
    return false;
  }
  bool Notify(const std::string& s_name, T val)
  {
    if (map_func_.find(s_name) != map_func_.end()) {
      return map_func_[s_name](val);
    } else {
      LOG_WARN("Not registed func yet <{}>", s_name.c_str());
    }
    return false;
  }

private:
  std::map< std::string, std::function<bool(T)> > map_func_;

};

// spetialize - class <void>
template <>
class NotifyCbFunc<void>
{
public:
  ~NotifyCbFunc() = default;

  bool RegistCbFunc(const std::string& s_name, std::function<bool()> func)
  {
    if (map_func_.find(s_name) == map_func_.end()) {
      map_func_[s_name] = func;
      LOG_INFO("Regist func <{}>", s_name.c_str());
      return true;
    }
    LOG_DEBUG("Already func is registed <{}>", s_name.c_str());
    return false;
  }
  bool Notify(const std::string& s_name)
  {
    if (map_func_.find(s_name) != map_func_.end()) {
      return map_func_[s_name]();
    } else {
      LOG_WARN("Not registed func yet <{}>", s_name.c_str());
    }
    return false;
  }

private:
  std::map< std::string, std::function<bool()> > map_func_;

};

} // namespace NVFR

#endif
