#include "core_astra/message/astra_message.h"

#include "common.pb.h"
#include "core_astra/action/action_base.h"
#include "core_astra/action/action_manager.h"
#include "util/logger.hpp"

#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>

namespace NaviFra {

void dispatch(const std::string& actionName, Poco::JSON::Object::Ptr req, const std::string& identity, const std::string& uuid)
{
    auto respond_error = [&](const std::string& msg) {
        Poco::JSON::Object::Ptr json = new Poco::JSON::Object;

        core_astra::REP rep;
        rep.set_uuid(uuid);
        rep.set_result("fail");
        json->set("message", msg);

        std::ostringstream oss;
        json->stringify(oss);
        rep.set_json_data(oss.str());

        std::string rep_bytes;
        rep.SerializeToString(&rep_bytes);

        zmq::message_t id_msg(identity.data(), identity.size());
        zmq::message_t empty_msg(0);
        zmq::message_t reply_msg(rep_bytes.data(), rep_bytes.size());

        ZMQHandler::instance().router().send(id_msg, zmq::send_flags::sndmore);
        ZMQHandler::instance().router().send(empty_msg, zmq::send_flags::sndmore);
        ZMQHandler::instance().router().send(reply_msg, zmq::send_flags::none);
    };

    // ActionManager 싱글턴 사용
    auto& am = ActionManager::instance();

    // 액션 초기화 안 되어 있으면 오류
    // (혹은 필요 시 initialize(ActionType::DEFAULT) 호출)
    if (!am.hasAction(actionName)) {
        respond_error("action not found");
        return;
    }

    ActionContext ctx;
    ctx.action_id = uuid;
    ctx.source = identity;
    ctx.respond = [identity, uuid](Poco::JSON::Object::Ptr json) {
        core_astra::REP rep;
        rep.set_uuid(uuid);
        rep.set_result(json->getValue<std::string>("result"));

        std::ostringstream oss;
        json->stringify(oss);
        rep.set_json_data(oss.str());

        std::string rep_bytes;
        rep.SerializeToString(&rep_bytes);

        zmq::message_t id_msg(identity.data(), identity.size());
        zmq::message_t empty_msg(0);
        zmq::message_t reply_msg(rep_bytes.data(), rep_bytes.size());

        ZMQHandler::instance().router().send(id_msg, zmq::send_flags::sndmore);
        ZMQHandler::instance().router().send(empty_msg, zmq::send_flags::sndmore);
        ZMQHandler::instance().router().send(reply_msg, zmq::send_flags::none);
    };

    am.getAction(actionName)->handle(ctx, std::move(req));
}

void handleRouterRequest(const void* sender, RouterMsg& ev)
{
    core_astra::REQ req;
    if (!req.ParseFromString(ev.payload))
        return;

    Poco::JSON::Parser parser;
    Poco::Dynamic::Var parsed = parser.parse(req.json_data());
    Poco::JSON::Object::Ptr reqObj = parsed.extract<Poco::JSON::Object::Ptr>();

    std::ostringstream oss;
    reqObj->stringify(oss);
    dispatch(req.action(), std::move(reqObj), ev.identity, req.uuid());
}

void onMessageCollector(const void* sender, const std::string& msg)
{
}

}  // namespace NaviFra
