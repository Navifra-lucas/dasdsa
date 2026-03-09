#include "nc_brain_agent/nc_brain_agent.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "rospack/rospack.h"

#include <Poco/ScopedLock.h>
#include <nc_brain_agent/ros/nc_ros_launch_manager.h>
#include <sys/wait.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

using namespace NaviFra;

NcROSLaunchManager::NcROSLaunchManager(NcROSLaunchManager const&)
    : activity_(this, &NcROSLaunchManager::runActivity)
{
}

NcROSLaunchManager::NcROSLaunchManager()
    : activity_(this, &NcROSLaunchManager::runActivity)
{
    // start();
}

NcROSLaunchManager::~NcROSLaunchManager()
{
    for (pid_t const& pid : pids_) {
        ::kill(pid, SIGINT);

        int32_t status;

        ::waitpid(pid, &status, 0);
    }
    // stop();
}

void NcROSLaunchManager::stop(pid_t const& pid, int32_t const& signal)
{
    Poco::FastMutex::ScopedLock scoped_lock(fastMutex_);

    auto pid_it = std::find(std::begin(pids_), std::end(pids_), pid);

    if (pid_it != pids_.end()) {
        ::kill(pid, signal);

        LOG_INFO("Stopping process with PID %d and signal %d", pid, signal);
    }
    else {
        LOG_ERROR("NcROSLaunchManager::stop - PID %d not found", pid);
    }
}

void NcROSLaunchManager::runActivity()
{
    // while (!activity_.isStopped()) {
    //     Poco::FastMutex::ScopedLock scoped_lock(fastMutex_);

    //     for (auto pid_it = std::begin(pids_); pid_it != std::end(pids_); ++pid_it) {
    //         pid_t const pid = *pid_it;

    //         int32_t status;

    //         if (::waitpid(pid, &status, WUNTRACED | WCONTINUED | WNOHANG) == pid) {
    //             if (WIFEXITED(status)) {
    //                 LOG_INFO("PID %d exited with status %d", pid, WEXITSTATUS(status));

    //                 pid_it = pids_.erase(pid_it);

    //                 if (pid_it == std::end(pids_)) {
    //                     break;
    //                 }
    //             }
    //             else if (WIFSIGNALED(status)) {
    //                 LOG_INFO("PID %d killed with signal %d", pid, WTERMSIG(status));

    //                 pid_it = pids_.erase(pid_it);

    //                 if (pid_it == std::end(pids_)) {
    //                     break;
    //                 }
    //             }
    //             else if (WIFSTOPPED(status)) {
    //                 LOG_INFO("PID %d stopped with signal %d", pid, WSTOPSIG(status));
    //             }
    //             else if (WIFCONTINUED(status)) {
    //                 LOG_INFO("PID %d continued", pid);
    //             }
    //         }
    //     }
    // }

    // Poco::FastMutex::ScopedLock scoped_lock(fastMutex_);

    for (pid_t const& pid : pids_) {
        ::kill(pid, SIGINT);

        int32_t status;

        ::waitpid(pid, &status, 0);
    }
}
