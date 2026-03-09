/**
 *
 * @brief NaviCore Comm
 * @details Core API
 * @author Kerry kerry@navifra.com
 * @date 2024-04-18
 * @version 0.0.1
 */

#ifndef NAVIFRA_NAVICORE_H
#define NAVIFRA_NAVICORE_H

#include <core_agent/data/types.h>

#include <string>
#include <vector>

namespace NaviFra {

enum NAVIFR_API
{
    NAVIFR_API_DEFAULT = 0,  // noetic
    NAVIFR_API_V1,
};

/**
 * @brief
 * @details
 * @return
 * @throws
 */
bool initailizeCore(NAVIFR_API api = NAVIFR_API::NAVIFR_API_DEFAULT);
/**
 * @brief
 * @details
 * @return
 * @throws
 */
void stopMapping();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void startMapping();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void stopNavigation();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void startNavigation();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void startSCAN();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
bool isSLAM();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void cancelTask();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void initialPose(Position position, Orientation orientation, bool correct_position);

void changeArea(std::string s_area_id);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void deviceGoal(std::string id, std::string name, std::string type);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void deviceCommand(std::string command);

/**
 * @brief
 * @details
 * @return
 * @throws
 */

void navifraCommand(std::string navifra_command);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void move(std::string id, std::string type);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void move(const std::vector<Pose>& paths);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void robotStop();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void taskCommand(std::string command);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void updateParameters();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void updateBrainMap(std::string type = "");

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void cmdVel(float linearX, float linearY, float angularZ);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void calibration(std::string calibrationType);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void updateRobotInfoPLC(std::string info);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void plcStartStatus(std::string msg);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void saveSLAM(int16_t isSave);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void playBackCMD(std::string cmd);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void repeatTestCMD(std::string cmd);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void startMotorStatus();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
bool getIsPLCManual();

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void docking(DockingNode start, DockingNode end, DockingType type);
void node_action();
/**
 * @brief
 * @details
 * @return
 * @throws
 */
ProcessResult requestStringSrv(std::string service, std::string data);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void calibrationDockingSave(std::string msg);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void barcodeReader(bool onOff);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void calibrationSteerWrite(std::string msg);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void calibrationSteerZeroSet(bool bZero);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void fabcolor(std::string color);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void fabsound(std::string id, int repeat_num);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void responseAvoidance(bool bPermission);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void disableLidar();

/**
 * @brief
 * @details
 * @return
 * @throws
 */

void SendNearestRobotPose(std::string pose);

/**
 * @brief
 * @details
 * @return
 * @throws
 */

void CheonilReadRegister(ReadRegister read_register);

/**
 * @brief
 * @details
 * @return
 * @throws
 */

void CheonilReadCoil(ReadCoil read_coil);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void NavifraSpeedLimit(float speed_limit);

/**
 * @brief
 * @details
 * @return
 * @throws
 */
void alarmclear();

}  // namespace NaviFra
#endif