#include "nc_wia_agent/nc_wia_agent.h"

#include <Poco/Base64Encoder.h>  // base64 인코딩
#include <Poco/DeflatingStream.h>  // gzip 압축
#include <core_agent/data/memory_repository.h>
#include <nc_wia_agent/action/nc_action_get_slam_map.h>
#include <nc_wia_agent/util/motor_feed_util.h>
#include <nc_wia_agent/util/motor_info_util.h>

using namespace NaviFra;

NcActionGetSlamMap::NcActionGetSlamMap()
{
}

NcActionGetSlamMap::~NcActionGetSlamMap()
{
}

std::string NcActionGetSlamMap::implName()
{
    return "NcActionGetSlamMap";
}

void NcActionGetSlamMap::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string s_map_path = Poco::Environment::get("HOME") + "navifra_solution/navicore/configs/map/latest";

    // json
    Poco::JSON::Object::Ptr map_data;
    std::ostringstream ostr;
    Poco::FileInputStream fis(s_map_path + "/map.json");
    Poco::StreamCopier::copyStream(fis, ostr);
    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(ostr.str());
    map_data = result.extract<Poco::JSON::Object::Ptr>();

    // // png
    // std::ifstream img(s_map_path + "/map.png", std::ios::binary);
    // std::vector<char> img_buffer((std::istreambuf_iterator<char>(img)), {});

    std::ifstream pgm_file(s_map_path + "/map.pgm", std::ios::binary);
    std::stringstream result_pgm;

    Poco::Base64Encoder base64_encoder(result_pgm);

    base64_encoder << pgm_file.rdbuf();
    base64_encoder.close();

    std::string base64_string = result_pgm.str();

    // // gzip 압축
    // std::ostringstream compressed_stream;
    // {
    //     Poco::DeflatingOutputStream gzip_stream(compressed_stream, Poco::DeflatingStreamBuf::STREAM_GZIP);
    //     gzip_stream.write(img_buffer.data(), img_buffer.size());
    //     gzip_stream.close();  // 반드시 닫아야 압축 스트림 flush 됨
    // }
    // std::string data = compressed_stream.str();  // 최종 gzip string

    // std::ostringstream base64_stream;
    // Poco::Base64Encoder encoder(base64_stream);
    // encoder << data;
    // encoder.close();

    // time
    // Poco::JSON::Object::Ptr stamp = new Poco::JSON::Object;
    // ros::Time now = ros::Time::now();
    // stamp->set("secs", now.sec);
    // stamp->set("nsecs", now.nsec);

    try {
        std::string rid = obj->getValue<std::string>("RID");
        int msg_id = obj->getValue<int>("msg_id");

        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;

        // 응답 필드
        response->set("resolution", map_data->getValue<float>("resolution"));  // resolution
        response->set("width", map_data->getValue<int>("width"));  // width
        response->set("height", map_data->getValue<int>("height"));  // height

        Poco::JSON::Object::Ptr position = new Poco::JSON::Object;
        position->set("x", 0.0);
        position->set("y", 0.0);
        position->set("z", 0.0);
        response->set("position", position);

        response->set("data", base64_string);

        // response->set("content_type", "gzip");
        response->set("map_version", "1.0");

        response->set("Cmd", "loadmapdb");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        NLOG(debug) << "Sending map message: " << oss.str();

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Exception in map_info: " << ex.displayText();
    }
}
