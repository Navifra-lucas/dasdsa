#include "nc_navican_remote/Utils/CommandClassifier.h"

using namespace NaviFra::NaviCAN::Remote;

// CommandClassifier 구현
std::unordered_map<std::string, CommandType> CommandClassifier::command_types_;

void CommandClassifier::initializeCommandTypes()
{
    if (!command_types_.empty())
        return;

    // 조회 명령 (Query Commands) - 상태 조회, 데이터 읽기
    command_types_["getPosition"] = CommandType::QUERY;
    command_types_["getSpeed"] = CommandType::QUERY;
    command_types_["getCurrent"] = CommandType::QUERY;
    command_types_["getVoltage"] = CommandType::QUERY;
    command_types_["getState"] = CommandType::QUERY;
    command_types_["getStatus"] = CommandType::QUERY;
    command_types_["isEnable"] = CommandType::QUERY;
    command_types_["isFault"] = CommandType::QUERY;
    command_types_["getMultiplePositions"] = CommandType::QUERY;
    command_types_["getMultipleSpeeds"] = CommandType::QUERY;
    command_types_["getMultipleCurrents"] = CommandType::QUERY;
    command_types_["getAllMotorData"] = CommandType::QUERY;
    command_types_["getCanBusState"] = CommandType::QUERY;
    command_types_["getErrorCode"] = CommandType::QUERY;
    command_types_["getErrorMessage"] = CommandType::QUERY;

    // 제어 명령 (Control Commands) - 단일 모터 제어
    command_types_["setTarget"] = CommandType::CONTROL;
    command_types_["enable"] = CommandType::CONTROL;
    command_types_["disable"] = CommandType::CONTROL;
    command_types_["shutdown"] = CommandType::CONTROL;
    command_types_["resetError"] = CommandType::CONTROL;
    command_types_["setMultipleTargets"] = CommandType::CONTROL;

    // 시스템 명령 (System Commands) - 전체 모터 제어, 시스템 레벨 작업
    command_types_["enableAll"] = CommandType::SYSTEM;
    command_types_["disableAll"] = CommandType::SYSTEM;
    command_types_["shutdownAll"] = CommandType::SYSTEM;
    command_types_["resetErrorAll"] = CommandType::SYSTEM;
    command_types_["presetEncoderAll"] = CommandType::SYSTEM;
    command_types_["emergencyStop"] = CommandType::SYSTEM;

    // 브로드캐스트 관련 (Broadcast) - 실시간 데이터 구독 관리
    command_types_["subscribeRealtime"] = CommandType::BROADCAST;
    command_types_["unsubscribeRealtime"] = CommandType::BROADCAST;
}

CommandType CommandClassifier::classifyCommand(const std::string& command)
{
    initializeCommandTypes();

    auto it = command_types_.find(command);
    if (it != command_types_.end()) {
        return it->second;
    }

    // 알 수 없는 명령어는 제어 명령으로 분류 (안전을 위해)
    return CommandType::CONTROL;
}