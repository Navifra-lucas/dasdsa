#ifndef NAVIFRA_NAVI_CORE_NOETIC_H
#define NAVIFRA_NAVI_CORE_NOETIC_H

#include "core_agent/core/navicore_base.h"

#include <Poco/Stopwatch.h>
#include <Poco/Timestamp.h>
#include <core_msgs/Graph.h>
#include <core_msgs/MotorInfo.h>
#include <core_msgs/NaviAlarm.h>
#include <core_msgs/NavicoreStatus.h>
#include <core_msgs/TaskAlarm.h>
#include <core_msgs/CheonilReadRegister.h>
#include <core_msgs/CheonilReadCoil.h>
#include <core_msgs/BatteryInfo.h>
#include <core_msgs/WiaForkInfo.h>
#include <task_msgs/Charging.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/param.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <regex>
#include <string>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace NaviFra {
using ROSService = std::map<std::string, ros::ServiceClient>;
using ROSServicePtr = std::unique_ptr<ROSService>;

class NaviCoreNoetic : public NaviCore {
public:
    NaviCoreNoetic();
    virtual ~NaviCoreNoetic();

    using Ptr = std::shared_ptr<NaviCoreNoetic>;

public:
    virtual void stopMapping() final;
    virtual void startMapping() final;
    virtual void stopNavigation() final;
    virtual void startNavigation() final;
    virtual void startSCAN() final;
    virtual bool isSLAM() final;
    virtual void cancelTask() final;
    virtual void initialPose(Position position, Orientation orientation, bool correct_position) final;
    virtual void deviceGoal(std::string id, std::string name, std::string type) final;
    virtual void deviceCommand(std::string command) final;
    virtual void navifraCommand(std::string navifra_command) final;
    virtual void robotStop() final;
    virtual void move(std::string id, std::string type) final;
    virtual void move(const std::vector<Pose>& paths) final;
    virtual void taskCommand(std::string command) final;
    virtual void updateParameters() final;
    virtual void updateBrainMap(std::string type) final;
    virtual void cmdVel(float linearX, float linearY, float angularZ) final;
    virtual void calibration(std::string calibrationType) final;
    virtual void updateRobotInfoPLC(std::string info) final;
    virtual void docking(DockingNode start, DockingNode end, DockingType type) final;
    virtual void node_action() final;
    virtual void plcStartStatus(std::string msg) final;
    virtual void saveSLAM(int16_t isSave) final;
    virtual void playBackCMD(std::string cmd) final;
    virtual void repeatTestCMD(std::string cmd) final;
    virtual void startMotorStatus() final { motorStatusWatch_.restart(); }
    virtual bool getIsPLCManual() final;
    virtual ProcessResult requestStringSrv(std::string service, std::string data) final;
    virtual void calibrationDockingSave(std::string msg) final;
    virtual void barcodeReader(bool onOff) final;
    virtual void calibrationSteerWrite(std::string msg) final;
    virtual void calibrationSteerZeroSet(bool bZero) final;
    virtual void fabcolor(std::string color) final;
    virtual void fabsound(std::string id, int repeat_num) final;
    virtual void responseAvoidance(bool bPermission) final;
    virtual void disableLidar() final;
    virtual void SendNearestRobotPose(std::string pose) final;
    virtual void alarmclear() final;
    virtual void CheonilReadRegister(ReadRegister read_register) final;
    virtual void CheonilReadCoil(ReadCoil read_coil) final;
    virtual void NavifraSpeedLimit(float speed_limit) final;

private:
    void onFrontCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onRearCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onLeftCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onRightCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onV2VCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onCameraCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onObsCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onIsSlam(const std_msgs::Bool::ConstPtr msg);
    void onMappingProgress(const std_msgs::Int16::ConstPtr msg);
    void onRobotPos(const geometry_msgs::PoseWithCovarianceStamped msg);
    void onNavifrainfo(const core_msgs::NavicoreStatus msg);
    void onTaskInfo(const std_msgs::String msg);
    std::vector<std::string> splitString(const std::string& str, char delimiter = '/');
    void onTaskAlarm(const core_msgs::TaskAlarm msg);
    void onTaskResponse(const core_msgs::TaskAlarm msg);
    void onErrorDist(const std_msgs::Float64 msg);
    void onMap(const nav_msgs::OccupancyGrid msg);
    void onSetSeverityMin(const std_msgs::Int16 msg);
    void onGlobalPath(const nav_msgs::Path path);
    void onLocalPath(const nav_msgs::Path path);
    void onPredictCollision(const geometry_msgs::PolygonStamped polygon);
    void onCollision(const geometry_msgs::PolygonStamped polygon);
    void onMotorInfo(const core_msgs::MotorInfo::ConstPtr msg);
    void onOdometry(const nav_msgs::Odometry::ConstPtr msg);
    void onObstacle(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void onCustomMessgae(const std_msgs::String::ConstPtr msg);
    void onParamUpdate(const std_msgs::String::ConstPtr msg);
    void onRecvAlarm(const core_msgs::NaviAlarm::ConstPtr& msg);
    void setParamFromYAML(const std::string& baseKey, const YAML::Node& node);
    std::string typeName(std::string node);

    ////////////////////////////////////////////////////////////////////////////grey
    void onCaliResult(const std_msgs::Float64MultiArray::ConstPtr msg);
    void onCaliProgressValue(const std_msgs::Float64MultiArray::ConstPtr msg);
    ////////////////////////////////////////////////////////////////////////////grey

    void onSetMarker(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void onSetReflectors(const std_msgs::String::ConstPtr& msg);

    ///////////////////////////////////////////////////////////////////////////
    void onMotorDriveServeOn(const std_msgs::String::ConstPtr msg);
    void onChatterCallback(const rosgraph_msgs::Log::ConstPtr msg);
    void onSLAMGraph(const std_msgs::String::ConstPtr msg);
    void onHardwareInfo(const std_msgs::String::ConstPtr msg);
    void onBatteryInfo(const core_msgs::BatteryInfo::ConstPtr& msg);
    void onRecvCharge(const std_msgs::Bool::ConstPtr& msg);
    void onRecvChargingCmd(const task_msgs::Charging::ConstPtr& msg);
    void onRecvLightingCmd(const std_msgs::Bool::ConstPtr& msg);
    void onRecvOssdFieldCmd(const std_msgs::Int16::ConstPtr& msg);
    void onChangeCurrentID(const std_msgs::String::ConstPtr msg);
    void updatePointCloud();
    void onPlcAlarmClear(const std_msgs::Bool::ConstPtr& msg);
    void onForkCmd(const core_msgs::WiaForkInfo::ConstPtr& msg);
    void onPallectID(const std_msgs::String::ConstPtr& msg);
    void onWiaNowTask(const std_msgs::String::ConstPtr& msg);

    void changeArea(std::string s_area_id);
    std::string generateUUID() 
    {
        boost::uuids::random_generator generator;
        boost::uuids::uuid uuid = generator();
        return boost::uuids::to_string(uuid);
    }
    
private:
    template <class MSG>
    bool sendROSMessage(std::string action, MSG msg)
    {
#ifndef CPP_UNIT_TEST
        if (rosPubs_.find(action) != rosPubs_.end()) {
            rosPubs_[action].publish(msg);
            return true;
        }

        return false;

#else
        return true;
#endif
    }

private:
    std::map<std::string, ros::Publisher> rosPubs_;
    std::vector<ros::Subscriber> rosSubs_;
    ROSServicePtr service_;

    bool isSLAM_;

    Poco::FastMutex fastMutexUpdatePointCloud_;

    Poco::Timestamp statusPubTime_;
    Poco::Stopwatch scanStartTime_;
    Poco::Stopwatch motorStatusWatch_;

    geometry_msgs::Twist controlMsg_;

    double timeFrontCloud_;
    double timeRearCloud_;
    double timeLeftCloud_;
    double timeRightCloud_;
    double timeV2VCloud_;
    double timeCameraCloud_;
    double timeObsCloud_;

    bool isRobotManualMode_;
    bool isPlcRemoteManual_;
    bool isUseLidar;
    std::string naviAlarm_ = "0";
    std::string s_current_node_id_ = "";
};
}  // namespace NaviFra
#endif
