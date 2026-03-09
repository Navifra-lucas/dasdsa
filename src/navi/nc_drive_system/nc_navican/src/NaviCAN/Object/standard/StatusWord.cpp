#include "NaviCAN/Object/standard/StatusWord.hpp"

namespace NaviFra {
namespace NaviCAN {
namespace Object {
namespace Standard {

StatusWord::StatusWord(uint16_t value)
    : BitField<uint16_t>(value)
{
}

bool StatusWord::isNotReadyToSwitchOn() const
{
    return (!getBit(READY_TO_SWITCH_ON) && 
    !getBit(SWITCHED_ON) && 
    !getBit(OPERATION_ENABLED) && 
    !getBit(FAULT) &&
    !getBit(SWITCH_ON_DISABLED));
}
bool StatusWord::isSwitchOnDisabled() const
{
    return (getBit(SWITCH_ON_DISABLED) && 
    !getBit(READY_TO_SWITCH_ON) && 
    !getBit(SWITCHED_ON) && 
    !getBit(OPERATION_ENABLED) && 
    !getBit(FAULT));
}
bool StatusWord::isReadyToSwitchOn() const
{
    return (getBit(READY_TO_SWITCH_ON) && 
    !getBit(SWITCHED_ON) && 
    !getBit(OPERATION_ENABLED) && 
    !getBit(FAULT) && 
    !getBit(SWITCH_ON_DISABLED) &&
    getBit(QUICK_STOP));
}
bool StatusWord::isSwitchedOn() const
{
    return (getBit(READY_TO_SWITCH_ON) && 
    getBit(SWITCHED_ON) && 
    !getBit(OPERATION_ENABLED) && 
    !getBit(FAULT) && 
    !getBit(SWITCH_ON_DISABLED) &&
    getBit(QUICK_STOP));
}
bool StatusWord::isOperationEnabled() const
{
    return (getBit(READY_TO_SWITCH_ON) && 
        getBit(SWITCHED_ON) && 
        getBit(OPERATION_ENABLED) && 
        !getBit(FAULT) && 
        !getBit(SWITCH_ON_DISABLED) &&
        getBit(QUICK_STOP));
}
bool StatusWord::isQuickStopActive() const
{
    return (getBit(READY_TO_SWITCH_ON) && 
        getBit(SWITCHED_ON) && 
        !getBit(OPERATION_ENABLED) && 
        !getBit(FAULT) && 
        !getBit(SWITCH_ON_DISABLED) && 
        !getBit(QUICK_STOP));
}
bool StatusWord::isFaultReactionActive() const
{
    return (!getBit(READY_TO_SWITCH_ON) && 
        !getBit(SWITCHED_ON) && 
        !getBit(OPERATION_ENABLED) && 
        getBit(FAULT) && 
        !getBit(SWITCH_ON_DISABLED));
}
bool StatusWord::isFault() const
{
    return getBit(FAULT);
}



}
}
}
}