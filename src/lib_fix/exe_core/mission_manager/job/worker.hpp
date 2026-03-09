
/*
 * @file	: worker.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: 할당 받은 job을 수행
 * @remark	: boost 의존성 있음
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_WORKER_HPP_
#define NAVIFRA_WORKER_HPP_

#include "util/util.hpp"

#include <boost/thread/thread.hpp>

#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

namespace NaviFra {
class Worker {
public:
    Worker();
    virtual ~Worker();

    void SetThread(const std::string& s_job_name, const std::function<void()>& pt_func, int n_period_ms);

    void Finalize();

    void FinalizeAsync();

    void SetPeriod(const int& n_period_ms);

    void StartWork();

    void PauseWorker();

    void ResumeWorker();

    bool CheckWorking();

    bool CheckTermination();

    std::string GetJobName();

private:
    void DoWork();

private:
    int n_period_ms_;

    bool b_proc_termination_;
    bool b_proc_terminated_;
    bool b_proc_pause_flag_;

    std::string s_job_name_;
    std::function<void()> pt_func_;
    std::mutex o_mutex_;

    boost::thread thread_;
};
}  // namespace NaviFra

#endif
