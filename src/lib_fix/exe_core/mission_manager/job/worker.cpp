#include "worker.hpp"

#include "util/logger.hpp"

namespace NaviFra {
Worker::Worker()
    : b_proc_termination_(true)
    , b_proc_pause_flag_(false)
    , b_proc_terminated_(true)
    , n_period_ms_(0)
{
    LOG_INFO("Worker constructor");
}

Worker::~Worker()
{
    LOG_INFO("Worker destructor");
}

void Worker::Finalize(void)
{
    if (CheckWorking() == false) {
        return;
    }
    else {
        std::unique_lock<std::mutex> o_lock(o_mutex_);
        b_proc_terminated_ = true;
    }
}

void Worker::FinalizeAsync()
{
    {
        std::unique_lock<std::mutex> o_lock(o_mutex_);
        b_proc_termination_ = true;
    }

    // while(b_proc_terminated_== false)
    // {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
}

void Worker::SetThread(const std::string& str_job_name, const std::function<void()>& pt_func, int n_period_ms)
{
    pt_func_ = pt_func;
    n_period_ms_ = n_period_ms;
    s_job_name_ = str_job_name;
}

void Worker::SetPeriod(const int& n_period_ms)
{
    n_period_ms_ = n_period_ms;
}

void Worker::StartWork()
{
    if (thread_.joinable()) {
        LOG_INFO("join");
        thread_.join();
    }

    LOG_INFO("%s job StartWork", s_job_name_.c_str());
    {
        std::unique_lock<std::mutex> o_lock(o_mutex_);
        b_proc_termination_ = false;
        b_proc_pause_flag_ = false;
        b_proc_terminated_ = false;
    }
    // boost::thread( boost::bind(&Worker::DoWork, this)  );
    thread_ = boost::thread(&Worker::DoWork, this);
}

void Worker::DoWork()
{
    LOG_INFO("This %s job DoWork", s_job_name_.c_str());
    while (false == b_proc_termination_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

        if (false == b_proc_pause_flag_) {
            pt_func_();
        }
        NaviFra::Util::SyncExecIntervalTime_(start, n_period_ms_);
    }

    {
        std::unique_lock<std::mutex> o_lock(o_mutex_);
        b_proc_terminated_ = true;
    }
    LOG_INFO("This %s job has been terminated", s_job_name_.c_str());
}

void Worker::PauseWorker()
{
    std::unique_lock<std::mutex> o_lock(o_mutex_);
    b_proc_pause_flag_ = true;
}

void Worker::ResumeWorker()
{
    std::unique_lock<std::mutex> o_lock(o_mutex_);
    b_proc_pause_flag_ = false;
}

bool Worker::CheckWorking()
{
    return (!b_proc_termination_);
}

bool Worker::CheckTermination()
{
    return (b_proc_terminated_);
}

std::string Worker::GetJobName()
{
    return s_job_name_;
}
}  // namespace NaviFra
