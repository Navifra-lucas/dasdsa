#ifndef NAVIFRA_ACTION_BASE_H
#define NAVIFRA_ACTION_BASE_H

#include <Poco/JSON/Object.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>

namespace NaviFra {

// 액션 분류(필요 없으면 단일 DEFAULT만 사용)
enum class ActionType
{
    DEFAULT,
    VOLVO
};

// 표준 에러 코드(프로젝트에 맞게 확장/축소)
enum class ActionError
{
    None = 0,
    InvalidArgument,
    NotFound,
    Conflict,
    Timeout,
    Cancelled,
    Internal
};

// 비동기 실행 컨텍스트: 브로커/라우터와 무관한 논리 정보만 보유
struct ActionContext {
    // 상관관계/추적용 ID
    std::string action_id;

    // 논리적 소스(토픽/라우팅키/송신자 등)
    std::string source;

    // 메타데이터(브로커 레벨에서 주입)
    std::unordered_map<std::string, std::string> headers;

    // 마감 시간/취소 토큰
    std::chrono::steady_clock::time_point deadline{};
    std::atomic_bool* cancelled{nullptr};

    // 결과 응답 콜백 (성공/실패 공용)
    // 표준 포맷 권장: { ok(bool), action(string), data(object)?, error_code(string)?, error_message(string)?, detail(object)? }
    std::function<void(Poco::JSON::Object::Ptr)> respond;

    // 진행 이벤트/중간 결과 알림(선택)
    std::function<void(const std::string& event, Poco::JSON::Object::Ptr)> emit;

    // 헬퍼: 성공 응답
    void ok(const std::string& action, Poco::JSON::Object::Ptr data = nullptr) const
    {
        Poco::JSON::Object::Ptr res = new Poco::JSON::Object;
        res->set("result", "success");
        if (data)
            res->set("data", data);
        if (respond)
            respond(res);
    }

    // 헬퍼: 에러 응답
    void error(const std::string& action, ActionError code, const std::string& message, Poco::JSON::Object::Ptr detail = nullptr) const
    {
        Poco::JSON::Object::Ptr res = new Poco::JSON::Object;
        res->set("result", "fail");
        res->set("error_code", toString(code));
        res->set("message", message);
        if (detail)
            res->set("detail", detail);
        if (respond)
            respond(res);
    }

    static const char* toString(ActionError e)
    {
        switch (e) {
            case ActionError::None:
                return "none";
            case ActionError::InvalidArgument:
                return "invalid_argument";
            case ActionError::NotFound:
                return "not_found";
            case ActionError::Conflict:
                return "conflict";
            case ActionError::Timeout:
                return "timeout";
            case ActionError::Cancelled:
                return "cancelled";
            case ActionError::Internal:
                return "internal";
        }
        return "unknown";
    }
};

// 순수 가상 베이스: 오직 handle()만 구현하면 됨
class ActionBase {
public:
    using Ptr = std::shared_ptr<ActionBase>;
    virtual ~ActionBase() = default;

    // 새 API(유일한 확장 포인트): 반드시 오버라이드
    virtual void handle(ActionContext& ctx, Poco::JSON::Object::Ptr req) = 0;

    // 액션 명(선택) – 없으면 레지스트리 등록명으로 사용
    virtual std::string name() const { return ""; }
};

// 액션 레지스트리: 이름/타입으로 팩토리 등록
class ActionRegistry {
public:
    using CreateFunc = std::function<ActionBase::Ptr()>;

    static ActionRegistry& instance()
    {
        static ActionRegistry inst;
        return inst;
    }

    void registerAction(ActionType type, const std::string& actionName, CreateFunc func) { actionMap_[type][actionName] = std::move(func); }

    const std::map<ActionType, std::map<std::string, CreateFunc>>& getActions() const { return actionMap_; }

private:
    ActionRegistry() = default;
    std::map<ActionType, std::map<std::string, CreateFunc>> actionMap_;
};

// 등록 매크로 (사용 예: REGISTER_ACTION("barcode_reader", NcActionBarcodeReader, ActionType::DEFAULT))
#define REGISTER_ACTION(ActionNameLiteral, ActionClass, ActionTypeEnum)                               \
    namespace {                                                                                       \
    struct ActionClass##Registrator {                                                                 \
        ActionClass##Registrator()                                                                    \
        {                                                                                             \
            ActionRegistry::instance().registerAction(                                                \
                ActionTypeEnum, ActionNameLiteral, []() { return std::make_shared<ActionClass>(); }); \
        }                                                                                             \
    };                                                                                                \
    static ActionClass##Registrator global_##ActionClass##Registrator;                                \
    }

}  // namespace NaviFra

#endif  // NAVIFRA_ACTION_BASE_H
