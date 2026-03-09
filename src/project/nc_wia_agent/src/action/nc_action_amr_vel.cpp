#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_amr_vel.h>
#include <std_msgs/Float64.h>

using namespace NaviFra;

NcActionAMRVel::NcActionAMRVel()
{
    max_speed_pub_ = nh.advertise<std_msgs::Float64>("/navifra/speed_limit", 10);
}

NcActionAMRVel::~NcActionAMRVel()
{
}

std::string NcActionAMRVel::implName()
{
    return "NcActionAMRVel";
}

void NcActionAMRVel::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");
        double value = obj->getValue<double>("value");

        NLOG(info) << "AMR Velocity request: RID = " << rid << ", value = " << value;

        std_msgs::Float64 msg;
        msg.data = value;
        max_speed_pub_.publish(msg);

        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "amr_vel");
        response->set(
            "msg_time",
            Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));  // 그대로 사용하거나 Timestamp로 갱신 가능
        response->set("msg_id", 0);
        response->set("RID", rid);
        response->set("AmrId", rid);
        response->set("Result", "S");
        response->set("message", Poco::format("%.1f", value));  // 응답 확인용 메시지

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "AMRVelocity Exception: " << ex.displayText();
    }
}