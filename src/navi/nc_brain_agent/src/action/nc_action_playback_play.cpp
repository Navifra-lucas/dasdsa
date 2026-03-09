#include "nc_brain_agent/nc_brain_agent.h"

#include <boost/algorithm/string.hpp>
#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_playback_play.h>
#include <nc_brain_agent/service/nc_playback_service.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>

using namespace NaviFra;

NcActionPlaybackPlay::NcActionPlaybackPlay()
{
}

NcActionPlaybackPlay::~NcActionPlaybackPlay()
{
}

void NcActionPlaybackPlay::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    Poco::JSON::Object::Ptr data = obj->getObject("data");

    Poco::JSON::Object message;
    std::string empty = data->get("file").convert<std::string>();
    // boost::algorithm::replace_all(empty, ".active", "");

    NcPlaybackService::get().start();
    std::string homePath = std::getenv("HOME");
    std::string core_root = std::getenv("CORE_ROOT") ? std::getenv("CORE_ROOT") : "";
    std::string playback_path = core_root.empty() != true ? core_root + "/bag/" : homePath + "/navifra_solution/navicore/configs/bag/";
    // ROSbag 파일 열기
    rosbag::Bag bag;
    try {
        bag.open(playback_path + empty, rosbag::bagmode::Read);
    }
    catch (rosbag::BagIOException e) {
        NLOG(error) << e.what();
        sendResponseSuccess(source, obj->get("uuid"), "fail", "playback file open error");
        return;
    }
    NLOG(info) << "empty " << empty;
    playBackCMD("play/" + empty);

    // ROSbag 파일의 재생 시간 가져오기
    rosbag::View view(bag);
    ros::Time start_time = view.getBeginTime();
    ros::Time end_time = view.getEndTime();
    bag.close();
    // 재생 시간 출력
    // ROSbag 파일 닫기

    double duration = (end_time - start_time).toSec();

    message.set("start_position", start_time.toSec() - data->get("position").convert<double>());  // test 결과 뺄 시 정상적 시간
    //출력 받는 값 확인 후 결정 필요 message.set("start_position", start_time.toSec());
    message.set("duration", duration);  // total
    // duration in second set
    sendResponseSuccessWithData(source, obj->get("uuid").convert<std::string>(), message);
}

std::string NcActionPlaybackPlay::implName()
{
    return "NcActionPlaybackPlay";
}
