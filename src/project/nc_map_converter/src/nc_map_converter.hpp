#ifndef NC_MAP_CONVERTER_HPP_
#define NC_MAP_CONVERTER_HPP_

#include "core/util/logger.hpp"
#include "core_msgs/JsonList.h"
#include "core_msgs/MapJson.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nc_map_converter/net/nc_rest_api_client.h"
#include "nc_map_converter/redis/nc_redis_reader.h"
#include "opencv2/opencv.hpp"
#include "util/json_stream.hpp"
#include "yaml-cpp/yaml.h"

#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/Redis/Array.h>
#include <Poco/StreamCopier.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <jsoncpp/json/json.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>

#include <cmath>
#include <fstream>

// 'using namespace std;'는 헤더 파일에서 전역으로 사용하기보다 .cpp 파일에서 사용하는 것이 권장됩니다.
namespace NaviFra {
class MapConverter {
public:
    ~MapConverter();
    MapConverter(ros::NodeHandle& nh, ros::NodeHandle& nhp);

private:
    // ROS 관련 멤버
    ros::NodeHandle& nh_;
    ros::NodeHandle& nhp_;
    ros::Publisher pcd_to_png_pub_;
    ros::Subscriber map_request_sub_;
    ros::Subscriber map_change_sub_;
    ros::Subscriber map_core_to_brain_sub_;
    ros::Subscriber map_brain_to_core_sub_;
    ros::Subscriber map_brain_upload_sub_;
    ros::Subscriber map_brain_acs_update_sub_;

    // 경로 관련 멤버
    std::string s_map_info_folder_path_;
    std::string s_map_default_select_;
    std::string s_json_output_path_;
    std::string s_brain_json_path_;

    // 데이터 변환 관련 멤버
    Json::Value js_jsonObject_;
    nav_msgs::OccupancyGrid msg_map_image_;

    // ROS 콜백 함수
    void MapRequestCallback(const std_msgs::String::ConstPtr& msg);
    void MapChangeCallback(const std_msgs::String::ConstPtr& msg);
    void MapConvertCoreToBrainCallback(const std_msgs::String::ConstPtr& msg);
    void MapConvertBrainToCoreCallback(const std_msgs::String::ConstPtr& msg);
    void MapConvertBrainUploadCallback(const std_msgs::String::ConstPtr& msg);
    void MapConvertACSCallback(const std_msgs::Bool::ConstPtr& msg);

    // 핵심 로직 함수
    void MapConvertCoreToBrain();
    void SaveMaptoJson(const core_msgs::MapDB data);
    void LoadBrainMapFromFile();
    void SaveJsontoMap();

    // 유틸리티 함수
    core_msgs::MapDB LoadCoreMapFromFile();
    void LoadJsonData();  // SaveMaptoJson 내부에서 호출됨
    void SaveYaml(float width, float height, float f_resolution);
    std::string getToken();
    Poco::JSON::Object::Ptr loadJSON(std::string filename);
    void APIConnect();
    std::string getBackendHostFromEnv();
    std::string getBackendPortFromEnv();
    std::string getRedisHostFromEnv();
    std::string getRedisPortFromEnv();
    std::string getRedisPassFromEnv();
    void MapConvertBrainToCore();
    // API 클라이언트
    NaviBrain::NcRESTAPIClient::Ptr apiClient_;
};
};  // namespace NaviFra

#endif