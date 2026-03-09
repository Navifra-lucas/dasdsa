#ifndef NAVIFRA_ACTION_MANAGER_H
#define NAVIFRA_ACTION_MANAGER_H

#include <Poco/Event.h>
#include <Poco/JSON/Object.h>
#include <Poco/SingletonHolder.h>
#include <core_agent/action/action_base.h>

namespace NaviFra {
class ActionManager {
public:
    static ActionManager& instance()
    {
        static Poco::SingletonHolder<ActionManager> sh;
        return *sh.get();
    }

    ActionManager();
    virtual ~ActionManager();

    void onAction(std::string source, Poco::JSON::Object::Ptr obj);

    const std::map<std::string, ActionBase::Ptr> getActions() const { return actions_; }

public:
    void initialize(NaviFra::ActionType type = NaviFra::ActionType::DEFAULT);
    void printActions();

protected:
    virtual void implInitialize(NaviFra::ActionType type = NaviFra::ActionType::DEFAULT);

protected:
    std::map<std::string, ActionBase::Ptr> actions_;
};

}  // namespace NaviFra
#endif