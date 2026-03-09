#include "core_agent/core_agent.h"

#include <core_agent/action/action_base.h>

using namespace NaviFra;

ActionBase::ActionBase()
{
}

ActionBase::~ActionBase()
{
}

void ActionBase::onAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    implonAction(source, obj);
}

void ActionBase::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
}

std::string ActionBase::name()
{
    return implName();
}