/*
 * @file	: publish_callback.hpp
 * @date	: Feb 3, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: publish callback
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef PUBLISH_CALLBACK_HPP_
#define PUBLISH_CALLBACK_HPP_

#include <iostream>
#include <map>
#include <functional>
#include <type_traits>

#include "logger/logger.h"
#include "utils/singleton_generator.hpp"

namespace NVFR {

template <typename... Args>
class PublishCb : public NaviFra::SingletonGenerator< PublishCb<Args ...> >
{
public:
  using FuncType = std::function<void(const Args& ...)>;
  PublishCb() { std::cout << "Construct PublishCb\n"; }
  virtual ~PublishCb()
  {
    map_func_.clear();
    std::cout << "Destruct PublishCb\n";
  }

  bool RegistCbFunc(int n_key, FuncType func)
  {
    if (map_func_.find(n_key) == map_func_.end()) {
      map_func_[n_key] = func;
      LOG_INFO("Regist func <{}>", n_key);
      return true;
    }
    LOG_DEBUG("Already func is registed <{}>", n_key);
    return false;
  }

  void Publish(int n_key, const Args& ... args)
  {
    if (map_func_.find(n_key) != map_func_.end()) {
      map_func_[n_key](args...);
    } else {
      LOG_WARN("Not registed func yet <{}>", n_key);
    }
  }

private:
  std::map< int, FuncType > map_func_;

};

template <>
class PublishCb<void> : public NaviFra::SingletonGenerator< PublishCb<void> >
{
public:
  using FuncType = std::function<void(void)>;
  PublishCb() { std::cout << "Construct PublishCb\n"; }
  virtual ~PublishCb()
  {
    map_func_.clear();
    std::cout << "Destruct PublishCb\n";
  }

  bool RegistCbFunc(int n_key, FuncType func)
  {
    if (map_func_.find(n_key) == map_func_.end()) {
      map_func_[n_key] = func;
      LOG_INFO("Regist func <{}>", n_key);
      return true;
    }
    LOG_DEBUG("Already func is registed <{}>", n_key);
    return false;
  }

  void Publish(int n_key)
  {
    if (map_func_.find(n_key) != map_func_.end()) {
      map_func_[n_key]();
    } else {
      LOG_WARN("Not registed func yet <{}>", n_key);
    }
  }

private:
  std::map< int, FuncType > map_func_;

};

} // namespace NVFR

#endif
