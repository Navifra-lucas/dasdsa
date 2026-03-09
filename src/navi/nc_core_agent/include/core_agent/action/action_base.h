#ifndef NAVIFRA_ACTION_BASE_H
#define NAVIFRA_ACTION_BASE_H

#include <Poco/JSON/Object.h>

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace NaviFra {
enum class ActionType
{
    DEFAULT,
    VOLVO
};

class ActionBase {
public:
    ActionBase();
    virtual ~ActionBase();

    void onAction(std::string source, Poco::JSON::Object::Ptr obj);

    using Ptr = std::shared_ptr<ActionBase>;

public:
    std::string name();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj);
    virtual std::string implName() { return ""; }
};

class ActionRegistry {
public:
    using CreateFunc = std::function<std::shared_ptr<ActionBase>()>;

    static ActionRegistry& instance()
    {
        static ActionRegistry instance;
        return instance;
    }

    void registerAction(ActionType type, const std::string& actionName, CreateFunc func) { actionMap[type][actionName] = func; }

    const std::map<ActionType, std::map<std::string, CreateFunc>>& getActions() const { return actionMap; }

private:
    ActionRegistry() = default;
    std::map<ActionType, std::map<std::string, CreateFunc>> actionMap;
};

// clang-format off
#define REGISTER_ACTION(ActionName, ActionClass, ActionType) \
    namespace { \
        struct ActionClass##Registrator { \
            ActionClass##Registrator() { \
                ActionRegistry::instance().registerAction(ActionType, ActionName, []() -> std::shared_ptr<ActionBase> { \
                    return std::make_shared<ActionClass>(); \
                }); \
            } \
        }; \
        static ActionClass##Registrator global_##ActionClass##Registrator; \
    }
// clang-format on
}  // namespace NaviFra

#endif