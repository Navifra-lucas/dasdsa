#pragma once

#include <mutex>
#include <string>

namespace NaviFra {

class TaskResultPublisher {
public:
    static TaskResultPublisher& instance();

    void setTaskContext(const std::string& rid, const std::string& task_id, int msg_id);
    void publish(std::string result);
    int getResultValue(std::string state);

private:
    TaskResultPublisher();
    ~TaskResultPublisher();
    TaskResultPublisher(const TaskResultPublisher&) = delete;
    void operator=(const TaskResultPublisher&) = delete;

    std::string rid_;
    std::string task_id_;
    int msg_id_;

    std::mutex mtx_;
};

}  // namespace NaviFra