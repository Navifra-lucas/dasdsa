#include "nc_map_converter.hpp"

#include <cstdlib>
#include <filesystem>

using Poco::JSON::Object;
using Poco::Redis::Array;
using namespace NaviFra;
using namespace NaviBrain;
using namespace Poco::Redis;
using namespace Poco::JSON;
using namespace std;
using Poco::Net::HTTPResponse;

MapConverter::MapConverter(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    nhp.getParam("map_info_folder_path", s_map_info_folder_path_);  // Core Map data가 저장되는 경로
    nhp.getParam("map_default_select", s_map_default_select_);  // Core Map Project 폴더 이름, default latest
    nhp.getParam("json_output_path", s_json_output_path_);  // 변환된 Brain map.json이 저장되는 경로
    nhp.getParam("brain_json_path", s_brain_json_path_);  // Brain Map data가 저장되는 경로

    pcd_to_png_pub_ = nh_.advertise<std_msgs::Bool>("/map_convert/pcd_to_png", 1);
    map_request_sub_ = nh_.subscribe("/map_request", 1, &MapConverter::MapRequestCallback, this);
    map_change_sub_ = nh_.subscribe("/map_change", 1, &MapConverter::MapChangeCallback, this);
    map_core_to_brain_sub_ = nh_.subscribe("/map_convert/core_to_brain", 10, &MapConverter::MapConvertCoreToBrainCallback, this);
    map_brain_to_core_sub_ = nh_.subscribe("/map_convert/brain_to_core", 10, &MapConverter::MapConvertBrainToCoreCallback, this);
    map_brain_upload_sub_ = nh_.subscribe("/map_convert/brainui_upload", 10, &MapConverter::MapConvertBrainUploadCallback, this);
    // map_brain_acs_update_sub_ = nh_.subscribe("/map_update_acs", 10, &MapConverter::MapConvertACSCallback, this);

    LOG_INFO("map_info_folder_path::%s ", s_map_info_folder_path_.c_str());
    LOG_INFO("map_default_select::%s ", s_map_default_select_.c_str());
}

MapConverter::~MapConverter()
{
    LOG_INFO("");
}

void MapConverter::MapRequestCallback(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("requested %s ", msg->data.c_str());
    if (msg->data.empty()) {
        LOG_INFO("Brain Map Update -> brain to core");
        SaveJsontoMap();  // brain map save -> brain to core
    }
    else if (msg->data == "latest") {
        LOG_INFO("Core UI Update!");
    }
    else {
        LOG_INFO("Core Map Update -> core to brain");
        // MapConvertCoreToBrain();  // core map save -> core to brain
        APIConnect();  // brain map upload
    }
}

void MapConverter::MapChangeCallback(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("Map Changed Call back - %s", msg->data.c_str());
    s_map_default_select_ = msg->data;
    if (s_map_default_select_.size() == 0) {
        s_map_default_select_ = "latest";
    }
}

void MapConverter::MapConvertCoreToBrainCallback(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("Map Convert Core To Brain Callback %s ", msg->data.c_str());
    MapConvertCoreToBrain();
}

void MapConverter::MapConvertBrainToCoreCallback(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("Map Convert Brain To Core Callback %s ", msg->data.c_str());
    SaveJsontoMap();
}

void MapConverter::MapConvertBrainUploadCallback(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("Map Convert Brain Upload Callback %s ", msg->data.c_str());
    APIConnect();
}

void MapConverter::MapConvertACSCallback(const std_msgs::Bool::ConstPtr& msg)
{
    LOG_INFO("Map Convert ACS Callback %d ", msg->data);
    // MapConvertCoreToBrain();
    APIConnect();
}

void MapConverter::APIConnect()
{
    std::vector<std::string> fileds;
    fileds.push_back("DEVICE");

    std::string backend_host = getBackendHostFromEnv();
    std::string backend_port = getBackendPortFromEnv();

    apiClient_.reset(new NcRESTAPIClient(backend_host, backend_port));
    std::string strToken = getToken();

    NLOG(info) << "token success";

    Poco::JSON::Object::Ptr mapObj = new Poco::JSON::Object;
    string s_file_path = s_map_info_folder_path_ + "/" + s_map_default_select_ + "/map_meta.json";

    std::ifstream mapFile(s_file_path);
    if (mapFile) {
        std::stringstream buffer;
        buffer << mapFile.rdbuf();
        mapFile.close();

        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(buffer.str());
        Poco::JSON::Object::Ptr parsedMap = result.extract<Poco::JSON::Object::Ptr>();

        for (const auto& key : *parsedMap) {
            mapObj->set(key.first, key.second);
        }

        // 추가 필드 붙이기
        mapObj->set("title", "test_mapconv");  // 원하는 title 입력
        mapObj->set("is_master", true);  // 필요시 true로 설정

        // 전송
        std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res =
            apiClient_->post("/scan-maps/create", *mapObj, strToken);

        std::string body = std::get<0>(res);
        Poco::Net::HTTPResponse::HTTPStatus status = std::get<1>(res);
        std::string reason = std::get<2>(res);

        if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
            NLOG(info) << "HTTP OK";
        }
        else {
            NLOG(info) << "HTTP error reason : " << reason.c_str();
        }
    }
    else {
        NLOG(info) << "map_meta.json 파일을 열 수 없습니다.";
    }
}

std::string MapConverter::getToken()
{
    const char* HOME_DIR = std::getenv("HOME");
    auto s_robot_ip = std::getenv("ROBOT_IP");
    std::string homeDIR = (HOME_DIR != NULL) ? HOME_DIR : "/root";
    std::string extention = ".json";
    std::string default_file_name = "nc_brain_agent_develop.json";
    std::string filepath_ = homeDIR + "/navifra_solution/navicore/configs/configs/";
    std::string filename_ = filepath_ + "nc_brain_agent_develop" + "_" + (s_robot_ip ? s_robot_ip : "localhost") + extention;
    NLOG(info) << "filename_ " << filename_;

    Object::Ptr config = loadJSON(filename_);
    return config->getObject("env")->get("client_token").convert<std::string>();
}

Poco::JSON::Object::Ptr MapConverter::loadJSON(std::string filename)
{
    if (Poco::File(filename).exists()) {
        std::ostringstream ostr;
        Poco::FileInputStream fis(filename);
        Poco::StreamCopier::copyStream(fis, ostr);
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(ostr.str());
        Poco::JSON::Object::Ptr data = result.extract<Poco::JSON::Object::Ptr>();

        return std::move(data);
    }
    return nullptr;
}

std::string MapConverter::getBackendHostFromEnv()
{
    const char* BACKEND_HOST = std::getenv("BACKEND_HOST");
    std::string backend_host = (BACKEND_HOST != NULL) ? BACKEND_HOST : "127.0.0.1";
    return backend_host;
}
std::string MapConverter::getBackendPortFromEnv()
{
    const char* BACKEND_PORT = std::getenv("BACKEND_PORT");
    std::string backend_port = (BACKEND_PORT != NULL) ? BACKEND_PORT : "5000";
    return backend_port;
}
std::string MapConverter::getRedisHostFromEnv()
{
    const char* REDIS_HOST = std::getenv("REDIS_HOST");
    std::string redis_host = (REDIS_HOST != NULL) ? REDIS_HOST : "127.0.0.1";
    return redis_host;
}
std::string MapConverter::getRedisPortFromEnv()
{
    const char* REDIS_PORT = std::getenv("REDIS_PORT");
    std::string redis_port = (REDIS_PORT != NULL) ? REDIS_PORT : "6379";
    return redis_port;
}
std::string MapConverter::getRedisPassFromEnv()
{
    const char* REDIS_PASS = std::getenv("REDIS_PASS");
    std::string redis_pass = (REDIS_PASS != NULL) ? REDIS_PASS : "navifra1@3$";
    return redis_pass;
}

void MapConverter::MapConvertCoreToBrain()
{
    LOG_INFO("map convert Core to Brain !!");
    core_msgs::MapDB o_map_db = LoadCoreMapFromFile();

    SaveMaptoJson(o_map_db);
}

void MapConverter::MapConvertBrainToCore()
{
    LOG_INFO("map convert Brain to Core !!");
    // LoadBrainMapFromFile();
}

/**
 * @brief  Core UI의 map.png와 map.yaml 파일을 불러와 map data를 저장 (joy)
 *
 * @return MapDB
 */
core_msgs::MapDB MapConverter::LoadCoreMapFromFile()
{
    core_msgs::MapDB o_result;
    string s_file_path = s_map_info_folder_path_ + "/" + s_map_default_select_;

    std::ifstream fin(s_file_path + "/map.yaml");
    YAML::Node doc = YAML::Load(fin);

    float res;
    float origin[3];

    try {
        res = stof(doc["resolution"].as<std::string>().c_str());
    }
    catch (...) {
    }

    try {
        origin[0] = stof(doc["origin"][0].as<std::string>().c_str());
        origin[1] = stof(doc["origin"][1].as<std::string>().c_str());
        origin[2] = stof(doc["origin"][2].as<std::string>().c_str());
    }
    catch (...) {
    }

    LOG_INFO("Load ");

    cv::Mat img = cv::imread(s_file_path + "/map.png", 0);

    float f_map_origin_x = *(origin);
    float f_map_origin_y = *(origin + 1);

    float f_map_min_x = 1000;
    float f_map_min_y = 1000;
    float f_map_max_x = 0;
    float f_map_max_y = 0;

    LOG_INFO("Copy pixel data");

    // initialize value to unknwon
    msg_map_image_.header.frame_id = "map";
    msg_map_image_.info.width = img.cols;
    msg_map_image_.info.height = img.rows;
    msg_map_image_.info.resolution = res;
    msg_map_image_.info.origin.position.x = f_map_origin_x;
    msg_map_image_.info.origin.position.y = f_map_origin_y;
    msg_map_image_.info.origin.orientation.w = 1;
    msg_map_image_.data.resize(img.cols * img.rows, -1);
    for (int col = 0; col < img.cols; col++) {
        for (int row = 0; row < img.rows; row++) {
            int n_data = static_cast<int>(img.at<uchar>(row, col));
            if (n_data <= 40) {
                geometry_msgs::Pose o_pos;
                o_pos.position.x = float(col) * res + f_map_origin_x;
                o_pos.position.y = float(img.rows - row) * res + f_map_origin_y;
                if (f_map_min_x > o_pos.position.x)
                    f_map_min_x = o_pos.position.x;
                if (f_map_min_y > o_pos.position.y)
                    f_map_min_y = o_pos.position.y;
                if (f_map_max_x < o_pos.position.x)
                    f_map_max_x = o_pos.position.x;
                if (f_map_max_y < o_pos.position.y)
                    f_map_max_y = o_pos.position.y;
                o_result.map_db.poses.push_back(o_pos);
            }
        }
    }

    o_result.map_min_x = f_map_min_x;
    o_result.map_min_y = f_map_min_y;
    o_result.map_max_x = f_map_max_x;
    o_result.map_max_y = f_map_max_y;
    LOG_INFO("width, height : %d, %d", msg_map_image_.info.width, msg_map_image_.info.height);
    LOG_INFO("o_result %.2f %.2f %.2f %.2f", o_result.map_min_x, o_result.map_min_y, o_result.map_max_x, o_result.map_max_y);
    return o_result;
}
/**
 * @brief  map data를 받아서 map.json occupied_space에 추가하고 저장 (joy)
 */
void MapConverter::SaveMaptoJson(const core_msgs::MapDB map_data)
{
    string s_json_output_path = s_map_info_folder_path_ + "/" + s_map_default_select_;

    js_jsonObject_["id"] = "12345678test";  // brain ui 저장시 생성되는 값 채우지 않아도됨
    js_jsonObject_["floor"] = 1;  // brain ui 저장시 생성되는 값 채우지 않아도됨
    js_jsonObject_["revision"] = 1;  // brain ui 저장시 생성되는 값 채우지 않아도됨
    js_jsonObject_["title"] = "Hyundai";  // brain ui 저장시 생성되는 값 채우지 않아도됨
    js_jsonObject_["is_master"] = false;  // brain ui 저장시 생성되는 값 채우지 않아도됨
    js_jsonObject_["width"] =
        float(msg_map_image_.info.width) * float(msg_map_image_.info.resolution);  //+ 10.00; // brain UI에서 여백 요청, 10 더하기
    js_jsonObject_["height"] = float(msg_map_image_.info.height) * float(msg_map_image_.info.resolution);  //+ 10.00;
    js_jsonObject_["resolution"] = msg_map_image_.info.resolution;
    js_jsonObject_["offset_x"] = 0;
    js_jsonObject_["offset_y"] = 0;

    Json::Value jsonArray;

    for (const auto& pose : map_data.map_db.poses) {
        Json::Value positionArray(Json::arrayValue);
        positionArray.append(pose.position.x);
        positionArray.append(pose.position.y);
        positionArray.append(pose.position.z);

        jsonArray.append(positionArray);
    }
    js_jsonObject_["occupied_space"] = jsonArray;  // point cloud 좌표를 저장

    // LoadJsonData();

    std::ofstream out(s_json_output_path_ + "/map.json");
    LOG_INFO("map.json path %s/map.json", s_json_output_path_.c_str());
    if (out.is_open()) {
        Json::StreamWriterBuilder writer;
        writer.settings_["precision"] = 5;  // 소수점 자리수 제한
        std::string jsonString = Json::writeString(writer, js_jsonObject_);

        out << jsonString;

        out.close();

        LOG_INFO("JSON file was successfully created.");
    }
    else {
        LOG_ERROR("Unable to open file.");
    }
}

void MapConverter::LoadBrainMapFromFile()
{
    // 경로 변수 선언

    std::string s_json_path = s_brain_json_path_ + "/map.json";
    std::string s_pcd_path = s_map_info_folder_path_ + "/" + s_map_default_select_ + "/map.pcd";

    try {
        JsonStream o_json_stream;
        o_json_stream.ReadJsonFromFile(s_json_path);
        rapidjson::Document& doc = o_json_stream.GetJsonFile();

        if (doc.HasParseError()) {
            throw std::runtime_error("map.json parsing failed");
        }

        const auto& db_list = doc["occupied_space"];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->width = db_list.Size();
        cloud->height = 1;

        for (const auto& db_item : db_list.GetArray()) {
            pcl::PointXYZ point;
            point.x = db_item[0].GetFloat();
            point.y = db_item[1].GetFloat();
            point.z = db_item[2].GetFloat();
            cloud->points.push_back(point);
        }

        if (db_list.Size() != 0) {
            pcl::io::savePCDFileASCII(s_pcd_path, *cloud);
            NLOG(info) << "pcd 저장 완료";
        }
        else {
            NLOG(info) << "pcd 데이터가 없어 저장하지 않음";
        }
        // [4] 동일한 doc을 사용해 맵 메타 정보 → ROS param 등록
        float f_width = doc["width"].GetFloat();
        float f_height = doc["height"].GetFloat();
        float f_resolution = round(doc["resolution"].GetFloat() * 100) / 100;
        f_resolution = 0.05;  // 무조건 고정값

        ros::param::set("pcd_path", s_pcd_path);
        ros::param::set("resolution", f_resolution);
        ros::param::set("image_width", f_width / f_resolution);
        ros::param::set("image_height", f_height / f_resolution);

        SaveYaml(f_width, f_height, f_resolution);
    }
    catch (const std::exception& e) {
        NLOG(error) << "Error during map conversion: " << e.what();
    }
}

void MapConverter::SaveYaml(float f_width, float f_height, float f_resolution)
{
    string s_yaml_output_path = s_map_info_folder_path_ + "/" + s_map_default_select_ + "/map.yaml";
    YAML::Node yaml;

    yaml["origin"][0] = -f_width / 2;
    yaml["origin"][1] = -f_height / 2;
    yaml["origin"][2] = 0.0;
    yaml["negate"] = 0;
    yaml["occupied_thresh"] = 0.65;
    yaml["free_thresh"] = 0.1;
    yaml["robot_size"] = 0.5;
    yaml["node_size"] = 0.5;
    yaml["image"] = "map.png";
    yaml["resolution"] = 0.05;

    std::ofstream file(s_yaml_output_path);
    file << yaml;
    file.close();

    LOG_INFO("YAML file has been created.");
}

void MapConverter::SaveJsontoMap()
{
    string s_map_output_path = s_map_info_folder_path_ + "/" + s_map_default_select_;

    LoadBrainMapFromFile();

    std_msgs::Bool msg;
    msg.data = true;
    pcd_to_png_pub_.publish(msg);

    std::string source_path = s_brain_json_path_ + "/map.json";
    std::string destination_path = s_map_output_path + "/map_meta.json";

    try {
        std::filesystem::copy(source_path, destination_path, std::filesystem::copy_options::overwrite_existing);
        LOG_INFO("SUCCESS: map_meta.json was created by copying map.json");
    }
    catch (const std::filesystem::filesystem_error& e) {
        LOG_ERROR("FAIL: Could not copy map.json. Error: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_map_converter", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    NaviFra::MapConverter ms(nh, nhp);
    try {
        ros::spin();
    }
    catch (std::runtime_error& e) {
        LOG_ERROR("map_converter exception: %s", e.what());
        return -1;
    }
    return 0;
}