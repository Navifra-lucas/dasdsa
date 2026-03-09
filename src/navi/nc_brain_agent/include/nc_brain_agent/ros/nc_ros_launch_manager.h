#ifndef NC_ROS_LAUNCH_MANAGER_H
#define NC_ROS_LAUNCH_MANAGER_H

#include <Poco/Activity.h>
#include <Poco/Process.h>

#include <algorithm>
#include <numeric>

namespace NaviFra {
class NcROSLaunchManager {
public:
    NcROSLaunchManager(NcROSLaunchManager const&);
    NcROSLaunchManager();

    ~NcROSLaunchManager();
    using Ptr = std::shared_ptr<NcROSLaunchManager>;

public:
    template <typename... Args>
    pid_t start(Args... args)
    {
        std::vector<std::string> args_vector = {args...};

        if (args_vector.size() > 0) {
            //  pid_t pid = ::fork();
            //
            //  if (pid == 0) {
            //      ::setsid();
            //
            //      ::signal(SIGINT, SIG_IGN);
            //
            //      ::fclose(stdout);
            //      ::fclose(stdin);
            //      ::fclose(stderr);
            //
            //      ::execlp("roslaunch", "roslaunch", args..., nullptr);
            //  }
            //  else {
            Poco::FastMutex::ScopedLock scoped_lock(fastMutex_);

            std::string args_string = std::accumulate(
                std::next(std::begin(args_vector)), std::end(args_vector), args_vector[0],
                [](std::string lhs, std::string rhs) -> std::string { return lhs + " " + rhs; });

            Poco::ProcessHandle ph = Poco::Process::launch("roslaunch", args_vector);

            LOG_TRACE("Starting \"roslaunch %s\" with PID %d", args_string.c_str(), ph.id());

            pids_.push_back(ph.id());
            //  }

            return ph.id();
        }
        else {
            throw std::runtime_error("ROSLaunchManager::start - No arguments provided");
        }
    }
    void stop(pid_t const& pid, int32_t const& signal);

private:
    void start();
    void stop();
    bool isStopped();

    Poco::Activity<NcROSLaunchManager> activity_;

public:
    std::vector<pid_t> pids_;
    Poco::FastMutex fastMutex_;

private:
    void runActivity();
};

inline bool NcROSLaunchManager::isStopped()
{
    return activity_.isStopped();
}

inline void NcROSLaunchManager::start()
{
    activity_.start();
}

inline void NcROSLaunchManager::stop()
{
    activity_.stop();
}
}  // namespace NaviFra
#endif