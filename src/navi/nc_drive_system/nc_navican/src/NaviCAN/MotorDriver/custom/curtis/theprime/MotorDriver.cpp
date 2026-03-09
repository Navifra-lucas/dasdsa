#include "NaviCAN/MotorDriver/custom/curtis/theprime/MotorDriver.h"

#include "NaviCAN/MotorDriver/MotorDriverRegistrar.h"

namespace NaviFra {
namespace NaviCAN {
namespace MotorDriver {
namespace Custom {
namespace curtis {
namespace ThePrime {
namespace RTV3000 {

bool MotorDriver::setTargetInfo(double target)
{
    return true;
}

bool MotorDriver::handleInit()
{
    return true;
}

void MotorDriver::handleRead()
{
}

void MotorDriver::handleWrite()
{
}

bool MotorDriver::handleShutdown()
{
    return true;
}

bool MotorDriver::handleHalt()
{
    return true;
}
bool MotorDriver::handleRecover()
{
    return true;
}

bool HydraulicsDriver::setTargetInfo(double target)
{
    return true;
}

bool HydraulicsDriver::handleInit()
{
    return true;
}

void HydraulicsDriver::handleRead()
{
}

void HydraulicsDriver::handleWrite()
{
}

bool HydraulicsDriver::handleShutdown()
{
    return true;
}

bool HydraulicsDriver::handleHalt()
{
    return true;
}
bool HydraulicsDriver::handleRecover()
{
    return true;
}

}  // namespace RTV3000
}  // namespace ThePrime
}  // namespace curtis
}  // namespace Custom
}  // namespace MotorDriver
}  // namespace NaviCAN
}  // namespace NaviFra