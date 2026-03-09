#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_amr_version.h>

using namespace NaviFra;

NcActionAMRVersion::NcActionAMRVersion()
{
}

NcActionAMRVersion::~NcActionAMRVersion()
{
}

std::string NcActionAMRVersion::implName()
{
    return "NcActionAMRVersion";
}

void NcActionAMRVersion::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        // 응답 객체 생성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;

        // 버전 정보 예시. 실제로는 로컬에서 가져오거나 Config에서 읽을 수도 있음
        Poco::JSON::Array::Ptr versionList = new Poco::JSON::Array;
        versionList->add("1.0");
        versionList->add("1.1");
        versionList->add("-1");
        versionList->add("1.2");

        response->set("version_info", versionList);
        response->set("Cmd", "amr_ver");
        response->set(
            "msg_time",
            Poco::DateTimeFormatter::format(
                Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));  // 그대로 사용하거나 Poco::Timestamp로 갱신 가능
        response->set("msg_id", 0);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "AMRVersion Exception: " << ex.displayText();
    }
}
