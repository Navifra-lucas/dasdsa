/*
 * @file	: loop_thread_handler.hpp
 * @date	: Feb 3, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Decision Maker class for path planner and motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef LOOP_THREAD_HANDLER_HPP_
#define LOOP_THREAD_HANDLER_HPP_

#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <functional>
#include <string>
#include <utility>
#include <map>

#include "utils/loop_thread.hpp"

namespace NVFR {

/**
 * @brief LoopThread를 string key를 통해 간편게 생성 및 관리
 * @note LoopThread : void(void) 형식의 어떤 함수를 일정주기로 지속적인 수행을 담당하는 스레드를 생성 및 관리함
 */
class LoopThreadHandler
{
public:
  LoopThreadHandler() = default;
  virtual ~LoopThreadHandler();

  /**
   * @brief LoopThread를 생성하고 s_key로 등록
   * @param s_key string key
   * @param loop_func 일정주기로 실행할 함수, void(void) 형식의 함수
   * @param tail_func Terminate 혹은 Stop 등을 통해 loop_func을 실행하는 루프를 빠져나온 후 실행할 함수, void(void) 형식의 함수, default : []{}
   */
  bool Regist(std::string s_key, std::function<void()> loop_func, std::function<void()> tail_func=[]{});
  bool Remove(std::string s_key);

  bool ThreadIsRunning(std::string s_key);

  bool SetPeriod(std::string s_key, size_t un_period_ms);
  bool Start(std::string s_key);
  /**
   * @brief loop_func을 실행하는 루프를 종료하고 스레드 또한 종료함, 만약 dead lock이 걸릴 수 있으면 unlock_func을 통해 풀 수 있음, 스레드가 안전하게 종료하고 Stop 함수를 마침(빠져나옴)
   * @param unlock_func dead lock을 풀기 위한 함수
   */
  bool Stop(std::string s_key, std::function<void()> unlock_func);

  /**
   * @brief loop_func을 실행하는 루프를 종료함, 함수 Stop()와는 다르게 스레드의 종료 신호만 주고 이 함수를 마침(빠져나옴)
   */
  bool Terminate(std::string s_key);

private:
  std::map< std::string, std::unique_ptr<LoopThread> > map_;

};

} // namespace NVFR

#endif
