
#include "job.hpp"

#include "util/logger.hpp"

namespace NaviFra {
Job::Job()
{
    //    LOG_INFO("");
}

Job::~Job()
{
    // LOG_INFO("");
}

bool Job::AddJob(const std::string& s_job_name, const std::function<void()>& pt_func, int n_period_ms)
{
    if (map_worker_.find(s_job_name) == map_worker_.end()) {
        map_worker_[s_job_name] = std::make_unique<Worker>();
        map_worker_[s_job_name]->SetThread(s_job_name, pt_func, n_period_ms);
        return true;
    }
    else {
        LOG_INFO("%s has already been created.", s_job_name.c_str());
        return false;
    }
}

bool Job::GetJob(const std::string& s_job_name, Worker*& result_proc)
{
    auto it = map_worker_.find(s_job_name);
    if (it == map_worker_.end()) {
        return false;
    }
    else {
        result_proc = it->second.get();
        return true;
    }
}

bool Job::StartJob(const std::string& s_job_name)
{
    TerminateJob(s_job_name);
    if (map_worker_.find(s_job_name) == map_worker_.end()) {
        return false;
    }
    else {
        b_wait_to_run_ = true;
        {
            // std::unique_lock<std::mutex> o_lock(o_mutex_);
            map_worker_[s_job_name]->StartWork();
        }
        while (b_wait_to_run_) {
            b_wait_to_run_ = !(map_worker_[s_job_name]->CheckWorking());
            LOG_INFO(
                "while... b_wait_to_run_ %s, CheckWorking %d CheckTermination %d\r", s_job_name.c_str(),
                map_worker_[s_job_name]->CheckWorking(), map_worker_[s_job_name]->CheckTermination());
            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
            std::this_thread::yield();
        }
        return true;
    }
}

bool Job::TerminateJob(const std::string& s_job_name)
{
    if (map_worker_.find(s_job_name) == map_worker_.end()) {
        return false;
    }
    else {
        // std::unique_lock<std::mutex> o_lock(o_mutex_);
        map_worker_[s_job_name]->FinalizeAsync();
        return true;
    }
}

void Job::TerminateAll()
{
    // std::map<std::string, Worker*>::iterator it;
    for (auto it = map_worker_.begin(); it != map_worker_.end(); it++) {
        if (it->second->CheckWorking()) {
            LOG_INFO("TerminateAll %s, %d\r", it->second->GetJobName().c_str(), it->second->CheckTermination());
            {
                // std::unique_lock<std::mutex> o_lock(o_mutex_);
                it->second->FinalizeAsync();
            }
        }
        else {
            LOG_INFO(
                "This job %s is terminated .. CheckTermination %d checkworking : %d\r", it->second->GetJobName().c_str(),
                it->second->CheckTermination(), it->second->CheckWorking());
        }
    }
}

void Job::JoinTerminationAllJob()
{
    // std::map<std::string, Worker*>::iterator it;
    constexpr double WAIT_DURATION_THRESHOLD_SEC = 1.0;
    for (auto it = map_worker_.begin(); it != map_worker_.end(); it++) {
        LOG_INFO(" %s, %d\r", it->second->GetJobName().c_str(), it->second->CheckTermination());
        std::chrono::steady_clock::time_point terminate_wait_point = std::chrono::steady_clock::now();

        while (it->second->CheckTermination() == false) {
            std::chrono::duration<double> sec = std::chrono::steady_clock::now() - terminate_wait_point;
            if (sec.count() > WAIT_DURATION_THRESHOLD_SEC) {
                LOG_INFO("Try to JoinTerminate %s but fail.. why? ", it->second->GetJobName().c_str());
                break;
            }
            std::this_thread::yield();
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
            // LOG_INFO("while... JoinTerminate %s, CheckTermination %d checkworking : %d\r",it->second->GetJobName().c_str(),
            // it->second->CheckTermination(), it->second->CheckWorking());
            {
                // std::unique_lock<std::mutex> o_lock(o_mutex_);
                it->second->FinalizeAsync();
            }
        }
        LOG_INFO("After waiting--JoinTerminate %s, %d\r", it->second->GetJobName().c_str(), it->second->CheckTermination());
    }
    LOG_INFO("JoinTerminationAllJob function completed!!\n");
}
}  // namespace NaviFra
