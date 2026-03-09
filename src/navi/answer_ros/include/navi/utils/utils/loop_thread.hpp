/*
 * @file	: loop_thread.hpp
 * @date	: Feb 3, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Decision Maker class for path planner and motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef LOOP_THREAD_HPP_
#define LOOP_THREAD_HPP_

#include <chrono>
#include <thread>
#include <functional>

namespace NVFR {

/**
 * @brief void(void) 형식의 어떤 함수를 일정주기로 지속적인 수행을 담당하는 스레드를 생성 및 관리함
 * @param loop_func 일정주기로 실행할 함수, void(void) 형식의 함수
 * @param tail_func Terminate 혹은 Stop 등을 통해 loop_func을 실행하는 루프를 빠져나온 후 실행할 함수, void(void) 형식의 함수, default : []{}
 */
class LoopThread
{
public:
  LoopThread(std::function<void()> loop_func, std::function<void()> tail_func=[]{});
  virtual ~LoopThread();

  bool ThreadIsRunning();

  void SetPeriod(size_t un_period_ms);
  void Start();

  /**
   * @brief loop_func을 실행하는 루프를 종료하고 스레드 또한 종료함, 만약 dead lock이 걸릴 수 있으면 unlock_func을 통해 풀 수 있음, 스레드가 안전하게 종료하고 Stop 함수를 마침(빠져나옴)
   * @param unlock_func dead lock을 풀기 위한 함수
   */
  void Stop(std::function<void()> unlock_func);

  /**
   * @brief loop_func을 실행하는 루프를 종료함, 함수 Stop()와는 다르게 스레드의 종료 신호만 주고 이 함수를 마침(빠져나옴)
   */
  void Terminate();

private:
  bool b_running_ = false;
  bool b_terminated_ = true;

  std::thread th_;
  std::function<void()> loop_func_;
  std::function<void()> tail_func_;
  std::chrono::milliseconds chr_period_ms_ = std::chrono::milliseconds(200UL);

  void LoopFunc();

};

} // namespace NVFR

#endif
