#ifndef NAVIFRA_NAVI_CORE_H
#define NAVIFRA_NAVI_CORE_H

#include <Poco/JSON/Object.h>
#include <core_agent/data/types.h>

#include <functional>

namespace NaviFra {
class NaviCore {
public:
    NaviCore() = default;
    virtual ~NaviCore() = default;

    using JSONCallback = std::function<void(Poco::JSON::Object::Ptr)>;

public:
    virtual void stopMapping() = 0;
    virtual void startMapping() = 0;
    virtual void stopNavigation() = 0;
    virtual void startNavigation() = 0;
    virtual void startSCAN() = 0;
    virtual bool isSLAM() = 0;
    virtual void cancelTask() = 0;
    virtual void initialPose(Position position, Orientation orientation, bool correct_position) = 0;
    virtual void changeArea(std::string s_area_id) = 0;
    virtual void deviceGoal(std::string id, std::string name, std::string type) = 0;
    virtual void deviceCommand(std::string command) = 0;
    virtual void navifraCommand(std::string navifra_command) = 0;
    virtual void move(std::string id, std::string type) = 0;
    virtual void move(const std::vector<Pose>& paths) = 0;
    virtual void robotStop() = 0;
    virtual void taskCommand(std::string command) = 0;
    virtual void updateParameters() = 0;
    virtual void updateBrainMap(std::string type) = 0;
    virtual void cmdVel(float linearX, float linearY, float angularZ) = 0;
    virtual void calibration(std::string calibrationType) = 0;
    virtual void updateRobotInfoPLC(std::string info) = 0;
    virtual void docking(DockingNode start, DockingNode end, DockingType type) = 0;
    virtual void node_action() = 0;
    virtual void plcStartStatus(std::string msg) = 0;
    virtual void saveSLAM(int16_t isSave) = 0;
    virtual void playBackCMD(std::string cmd) = 0;
    virtual void repeatTestCMD(std::string cmd) = 0;
    virtual void startMotorStatus() = 0;
    virtual bool getIsPLCManual() = 0;
    virtual ProcessResult requestStringSrv(std::string service, std::string data) = 0;
    virtual void calibrationDockingSave(std::string msg) = 0;
    virtual void barcodeReader(bool onOff) = 0;
    virtual void calibrationSteerWrite(std::string msg) = 0;
    virtual void calibrationSteerZeroSet(bool bZero) = 0;
    virtual void fabcolor(std::string color) = 0;
    virtual void fabsound(std::string id, int repeat_num) = 0;
    virtual void responseAvoidance(bool bPermission) = 0;
    virtual void disableLidar() = 0;
    virtual void SendNearestRobotPose(std::string pose) = 0;
    virtual void CheonilReadRegister(ReadRegister read_register) = 0;
    virtual void CheonilReadCoil(ReadCoil read_coil) = 0;
    virtual void alarmclear() = 0;
    virtual void NavifraSpeedLimit(float speed_limit) = 0;

protected:
    JSONCallback onLocalPath_;
    JSONCallback onGlobalPath_;
};
}  // namespace NaviFra
#endif