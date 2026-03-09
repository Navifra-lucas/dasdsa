#ifndef NC_TASK_MANAGER_TOPIC_H
#define NC_TASK_MANAGER_TOPIC_H
namespace NaviFra {
class NcTaskManagerTopic {
public:
    NcTaskManagerTopic();
    ~NcTaskManagerTopic();

    static const std::string TOPIC_TASK_RESPONSE;
    static const std::string TOPIC_TASK_ALARM;
    static const std::string TOPIC_TASK_INFO;
    static const std::string TOPIC_NAV_CMD;
    static const std::string TOPIC_LOADED;
    static const std::string TOPIC_HW_CMD;
    static const std::string TOPIC_HW_TASK;
    static const std::string TOPIC_NAVIFRA_INFO;
    static const std::string TOPIC_CURRENT_NODE;
    static const std::string TOPIC_TASK_NAME;
    static const std::string TOPIC_OUTPUT_COMMAND;
    static const std::string TOPIC_FORK_INFO;
};
}  // namespace NaviFra
#endif  // NC_TASK_MANAGER_TOPIC_H