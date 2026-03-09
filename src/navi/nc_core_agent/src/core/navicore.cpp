#include "core_agent/core/navicore.h"

#include "core_agent/core/navicore_base.h"
#include "core_agent/core/navicore_noetic.h"
#include "core_agent/core_agent.h"

namespace NaviFra {

using NaviCorePtr = std::shared_ptr<NaviCore>;
NaviCorePtr context;

bool isInitailize()
{
    if (context.get() == nullptr) {
        return false;
    }
    else {
        return true;
    }
}

bool initailizeCore(NAVIFR_API api)
{
    switch (api) {
        case NAVIFR_API::NAVIFR_API_DEFAULT:
        {
            auto noetic = std::make_shared<NaviCoreNoetic>();
            context = std::dynamic_pointer_cast<NaviCore>(noetic);
            NLOG(info) << "NaviFra API uses ROS noetic";
        } break;
        case NAVIFR_API::NAVIFR_API_V1:
            NLOG(info) << "NaviFra API Not implemented V1";
            break;
        default:
            break;
    }

    return true;
}

void stopMapping()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->stopMapping();
}

void startMapping()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->startMapping();
}

void stopNavigation()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->stopNavigation();
}

void startNavigation()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->startNavigation();
}

void startSCAN()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->startSCAN();
}

bool isSLAM()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    return context->isSLAM();
}

void cancelTask()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    return context->cancelTask();
}

void initialPose(Position position, Orientation orientation, bool correct_position)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->initialPose(position, orientation, correct_position);
}

void changeArea(std::string s_area_id)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->changeArea(s_area_id);
}

void deviceGoal(std::string id, std::string name, std::string type)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->deviceGoal(id, name, type);
}

void deviceCommand(std::string command)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->deviceCommand(command);
}

void navifraCommand(std::string navifra_command)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->navifraCommand(navifra_command);
}

void move(std::string id, std::string type)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->move(id, type);
}

void move(const std::vector<Pose>& paths)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->move(paths);
}

void robotStop()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->robotStop();
}

void taskCommand(std::string command)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->taskCommand(command);
}

void updateParameters()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->updateParameters();
}

void updateBrainMap(std::string type)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->updateBrainMap(type);
}

void cmdVel(float linearX, float linearY, float angularZ)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->cmdVel(linearX, linearY, angularZ);
}

void calibration(std::string calibrationType)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->calibration(calibrationType);
}

void updateRobotInfoPLC(std::string info)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->updateRobotInfoPLC(info);
}

void plcStartStatus(std::string msg)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->plcStartStatus(msg);
}

void saveSLAM(int16_t isSave)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->saveSLAM(isSave);
}

void playBackCMD(std::string cmd)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->playBackCMD(cmd);
}

void repeatTestCMD(std::string cmd)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->repeatTestCMD(cmd);
}

void startMotorStatus()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->startMotorStatus();
}

bool getIsPLCManual()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    return context->getIsPLCManual();
}

void docking(DockingNode start, DockingNode end, DockingType type)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->docking(start, end, type);
}

void node_action()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->node_action();
}

ProcessResult requestStringSrv(std::string service, std::string data)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    return context->requestStringSrv(service, data);
}

void calibrationDockingSave(std::string msg)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->calibrationDockingSave(msg);
}

void barcodeReader(bool onOff)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->barcodeReader(onOff);
}

void calibrationSteerWrite(std::string msg)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->calibrationSteerWrite(msg);
}

void calibrationSteerZeroSet(bool bZero)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->calibrationSteerZeroSet(bZero);
}

void fabcolor(std::string color)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->fabcolor(color);
}

void fabsound(std::string id, int repeat_num)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->fabsound(id, repeat_num);
}

void responseAvoidance(bool bPermission)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->responseAvoidance(bPermission);
}

void disableLidar()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->disableLidar();
}


void SendNearestRobotPose(std::string pose)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->SendNearestRobotPose(pose);
}

void CheonilReadRegister(ReadRegister read_register)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->CheonilReadRegister(read_register);
}

void CheonilReadCoil(ReadCoil read_coil)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->CheonilReadCoil(read_coil);
}

void NavifraSpeedLimit(float speed_limit)
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->NavifraSpeedLimit(speed_limit);
}

void alarmclear()
{
    if (isInitailize() != true)
        throw Poco::NullValueException("NaviFra API context not initailized");
    context->alarmclear();
}

}  // namespace NaviFra