
/*
 * @file	: job.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: 할당 받은 job관리
 * @remark	: boost 의존성 있음
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_JOB_HPP_
#define NAVIFRA_JOB_HPP_


#include "worker.hpp"

#include <functional>
#include <iostream>
#include <map>

namespace NaviFra {
class Job {
public:
    Job();
    virtual ~Job();

    bool AddJob(const std::string& s_job_name, const std::function<void()>& pt_func, int n_period_ms);
    bool GetJob(const std::string& s_job_name, Worker*& result_proc);
    bool StartJob(const std::string& s_job_name);
    bool TerminateJob(const std::string& s_job_name);

    void TerminateAll();
    void JoinTerminationAllJob();

private:
    std::mutex o_mutex_;
    bool b_wait_to_run_ = false;
    std::map<std::string, std::unique_ptr<Worker>> map_worker_;
};
}  // namespace NaviFra

#endif