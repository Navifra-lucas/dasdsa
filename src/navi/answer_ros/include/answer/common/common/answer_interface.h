#pragma once

namespace ANSWER {
class AnswerInterface {
private:
    /* data */
    virtual void Initialize() = 0;
    virtual void GenerateSubscriptions() = 0;
    virtual void GeneratePublishers() = 0;
    virtual void GenerateServices() = 0;
    virtual void RegisterCallbacks() = 0;

public:
    virtual ~AnswerInterface() = default;
    virtual void TerminateNode() = 0;
};

}  // namespace ANSWER