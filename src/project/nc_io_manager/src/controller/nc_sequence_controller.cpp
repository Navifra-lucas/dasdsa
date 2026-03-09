#include "nc_io_manager/controller/nc_sequence_controller.h"

#include "util/logger.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>

namespace NaviFra {

SequenceController::SequenceController(std::shared_ptr<DataHandler> data_handler, ros::NodeHandle nh)
    : data_handler_(data_handler)
    , nh_(nh)
{
    NLOG(info) << "SequenceController created";

    charge_state_pub_ = nh_.advertise<std_msgs::Bool>("/charge_state", 5);
}

SequenceController::~SequenceController()
{
    stopAllSequences();
    NLOG(info) << "SequenceController destroyed";
}

void SequenceController::addSequence(std::shared_ptr<Sequence> sequence)
{
    if (!sequence) {
        NLOG(info) << "[ERROR] Cannot add null sequence";
        return;
    }

    // 같은 이름의 시퀀스가 있으면 제거 후 추가
    removeSequence(sequence->name);

    sequences_[sequence->name] = sequence;
    NLOG(info) << "[SEQUENCE] Added sequence: " << sequence->name << " (" << sequence->steps.size() << " steps)"
               << " - Initial state: " << stateToString(sequence->state);
}

void SequenceController::removeSequence(const std::string& name)
{
    auto it = sequences_.find(name);
    if (it != sequences_.end()) {
        // 실행 중이면 먼저 중단
        if (it->second->state == SequenceState::RUNNING || it->second->state == SequenceState::WAITING) {
            stopSequence(name);
        }

        sequences_.erase(it);
        NLOG(info) << "[SEQUENCE] Removed sequence: " << name;
    }
}

void SequenceController::SetOutputCommand(const std::string& name)
{
    s_outputcommand_ = name;
}

bool SequenceController::startSequence(const std::string& name)
{
    auto it = sequences_.find(name);
    if (it == sequences_.end()) {
        NLOG(info) << "[ERROR] Sequence not found: " << name;
        return false;
    }

    auto& sequence = it->second;

    // 이미 실행 중인지 확인
    if (sequence->state == SequenceState::RUNNING || sequence->state == SequenceState::WAITING) {
        NLOG(info) << "[WARNING] Sequence already running: " << name;
        return false;
    }

    // 시퀀스 초기화 및 시작
    sequence->reset();
    sequence->state = SequenceState::IDLE;

    NLOG(info) << "[SEQUENCE] Starting sequence: " << name;
    return true;
}

void SequenceController::stopSequence(const std::string& name)
{
    auto it = sequences_.find(name);
    if (it == sequences_.end()) {
        NLOG(info) << "[WARNING] Cannot stop - sequence not found: " << name;
        return;
    }

    auto& sequence = it->second;

    if (sequence->state == SequenceState::RUNNING || sequence->state == SequenceState::WAITING) {
        abortSequence(sequence, "Manual stop requested");
    }
}

void SequenceController::stopAllSequences()
{
    NLOG(info) << "[SEQUENCE] Stopping all sequences...";

    for (auto& pair : sequences_) {
        if (pair.second->state == SequenceState::RUNNING || pair.second->state == SequenceState::WAITING) {
            abortSequence(pair.second, "Stop all requested");
        }
    }
}

void SequenceController::process()
{
    if (!data_handler_) {
        return;
    }

    // 모든 시퀀스를 순서대로 처리
    for (auto& pair : sequences_) {
        processSequence(pair.second);
    }
}

void SequenceController::processSequence(std::shared_ptr<Sequence> seq)
{
    // 디버그: 현재 상태 출력
    static std::map<std::string, SequenceState> last_states;
    if (last_states[seq->name] != seq->state) {
        NLOG(debug) << "Sequence '" << seq->name << "' state: " << stateToString(seq->state);
        last_states[seq->name] = seq->state;
    }

    switch (seq->state) {
        case SequenceState::COMPLETED:
            // 완료된 시퀀스는 아무 작업도 하지 않음
            break;

        case SequenceState::ABORTED:
            // 중단된 시퀀스는 아무 작업도 하지 않음
            break;

        case SequenceState::STOPPED:
            // 아무것도 하지 않음 - 수동 시작 대기
            break;

        case SequenceState::IDLE:
            startFirstStep(seq);  // 이제 수동으로 시작된 것만 처리
            break;

        case SequenceState::RUNNING:
            processRunningStep(seq);
            break;

        case SequenceState::WAITING:
            processWaitingStep(seq);
            break;

        case SequenceState::ERROR:
            handleErrorState(seq);
            break;

        default:
            break;
    }
}

void SequenceController::startFirstStep(std::shared_ptr<Sequence> seq)
{
    NLOG(info) << "[SEQUENCE] Starting sequence: " << seq->name;

    if (seq->steps.empty()) {
        NLOG(info) << "[ERROR] No steps defined in sequence: " << seq->name;
        abortSequence(seq, "No steps defined");
        return;
    }

    seq->current_step = 0;
    seq->state = SequenceState::RUNNING;
    seq->step_start_time = std::chrono::steady_clock::now();

    // 첫 번째 스텝 실행
    executeCurrentStep(seq);
}

void SequenceController::processRunningStep(std::shared_ptr<Sequence> seq)
{
    // 1. 타임아웃 체크
    if (checkStepTimeout(seq)) {
        std::string timeout_msg = "Step '" + seq->steps[seq->current_step].name + "' timed out";
        abortSequence(seq, timeout_msg);
        return;
    }

    // 2. 다음 스텝으로 넘어갈 조건 확인
    if (checkStepCondition(seq)) {
        advanceToNextStep(seq);
    }
    else {
        // 조건이 만족되지 않으면 WAITING 상태로 전환
        seq->state = SequenceState::WAITING;
    }
}

void SequenceController::processWaitingStep(std::shared_ptr<Sequence> seq)
{
    // 1. 타임아웃 체크
    if (checkStepTimeout(seq)) {
        std::string timeout_msg = "Step '" + seq->steps[seq->current_step].name + "' timed out while waiting";
        abortSequence(seq, timeout_msg);
        return;
    }

    // 2. 조건 재확인
    if (checkStepCondition(seq)) {
        NLOG(info) << "[SEQUENCE] Condition met for step: " << seq->steps[seq->current_step].name;
        advanceToNextStep(seq);
    }

    // 3. 대기 중 주기적 로그 (5초마다)
    static std::map<std::string, std::chrono::steady_clock::time_point> last_log_times;
    auto now = std::chrono::steady_clock::now();

    if (last_log_times.find(seq->name) == last_log_times.end()) {
        last_log_times[seq->name] = now;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_times[seq->name]).count();

    if (elapsed >= 5) {
        NLOG(info) << "[SEQUENCE] Still waiting for condition: " << seq->steps[seq->current_step].description;
        last_log_times[seq->name] = now;
    }
}

void SequenceController::handleErrorState(std::shared_ptr<Sequence> seq)
{
    NLOG(info) << "[ERROR] Sequence in error state: " << seq->name;

    // 에러 상태에서는 모든 출력을 안전 상태로
    emergencyStopAllOutputs();

    // 시퀀스 중단
    abortSequence(seq, "Error state detected");
}

void SequenceController::executeCurrentStep(std::shared_ptr<Sequence> seq)
{
    if (seq->current_step >= seq->steps.size()) {
        completeSequence(seq);
        return;
    }

    const auto& step = seq->steps[seq->current_step];

    NLOG(info) << "[SEQUENCE] Executing step " << (seq->current_step + 1) << "/" << seq->steps.size() << ": " << step.name;

    try {
        // 스텝의 액션 실행
        if (step.action) {
            step.action(data_handler_);
            NLOG(info) << "[SEQUENCE] Action executed: " << step.description;
        }

        // 스텝 시작 시간 기록
        seq->step_start_time = std::chrono::steady_clock::now();
    }
    catch (const std::exception& e) {
        std::string error_msg = "Exception in step '" + step.name + "': " + e.what();
        abortSequence(seq, error_msg);
    }
}

bool SequenceController::checkStepCondition(std::shared_ptr<Sequence> seq)
{
    if (seq->current_step >= seq->steps.size()) {
        return true;  // 마지막 스텝이면 완료
    }

    const auto& step = seq->steps[seq->current_step];

    try {
        if (step.condition) {
            bool result = step.condition(data_handler_);
            return result;
        }
        else {
            // 조건이 없으면 즉시 다음 스텝으로
            return true;
        }
    }
    catch (const std::exception& e) {
        std::string error_msg = "Exception checking condition for step '" + step.name + "': " + e.what();
        abortSequence(seq, error_msg);
        return false;
    }
}

bool SequenceController::checkStepTimeout(std::shared_ptr<Sequence> seq)
{
    if (seq->current_step >= seq->steps.size()) {
        return false;
    }

    const auto& step = seq->steps[seq->current_step];

    if (step.timeout_ms <= 0) {
        return false;  // 타임아웃 없음
    }

    auto elapsed = getStepElapsedTime(seq);

    if (elapsed >= step.timeout_ms) {
        NLOG(info) << "[TIMEOUT] Step '" << step.name << "' timed out after " << elapsed << "ms (limit: " << step.timeout_ms << "ms)";
        return true;
    }

    return false;
}

void SequenceController::advanceToNextStep(std::shared_ptr<Sequence> seq)
{
    seq->current_step++;

    if (seq->current_step >= seq->steps.size()) {
        // 모든 스텝 완료
        completeSequence(seq);
    }
    else {
        // 다음 스텝으로 이동
        seq->state = SequenceState::RUNNING;
        NLOG(info) << "[SEQUENCE] Advancing to step " << (seq->current_step + 1) << ": " << seq->steps[seq->current_step].name;

        executeCurrentStep(seq);
    }
}

void SequenceController::completeSequence(std::shared_ptr<Sequence> seq)
{
    seq->state = SequenceState::COMPLETED;
    NLOG(info) << "[SEQUENCE] ✓ Sequence COMPLETED: " << seq->name;

    // 완료 시 콜백 실행
    onSequenceCompleted(seq->name);
}

void SequenceController::abortSequence(std::shared_ptr<Sequence> seq, const std::string& reason)
{
    seq->state = SequenceState::ABORTED;
    NLOG(info) << "[SEQUENCE] ✗ Sequence ABORTED: " << seq->name << " - Reason: " << reason;

    // 중단 시 안전 처리
    emergencyStopForSequence(seq);

    // 중단 시 콜백 실행
    onSequenceAborted(seq->name, reason);
}

SequenceState SequenceController::getSequenceState(const std::string& name) const
{
    auto it = sequences_.find(name);
    return (it != sequences_.end()) ? it->second->state : SequenceState::ERROR;
}

std::vector<std::string> SequenceController::getRunningSequences() const
{
    std::vector<std::string> running_sequences;

    for (const auto& pair : sequences_) {
        if (pair.second->state == SequenceState::RUNNING || pair.second->state == SequenceState::WAITING) {
            running_sequences.push_back(pair.first);
        }
    }

    return running_sequences;
}

void SequenceController::printStatus() const
{
    NLOG(info) << "\n=== Sequence Controller Status ===";
    NLOG(info) << "Total sequences: " << sequences_.size();

    if (sequences_.empty()) {
        NLOG(info) << "No sequences defined";
    }
    else {
        for (const auto& pair : sequences_) {
            const auto& seq = pair.second;
            NLOG(info) << "Sequence [" << seq->name << "] " << stateToString(seq->state);

            if (seq->state == SequenceState::RUNNING || seq->state == SequenceState::WAITING) {
                NLOG(info) << " (Step " << (seq->current_step + 1) << "/" << seq->steps.size() << ": " << seq->steps[seq->current_step].name
                           << ")";

                auto elapsed = getStepElapsedTime(seq);
                NLOG(info) << " [" << elapsed << "ms]";
            }

            NLOG(info);
        }
    }

    auto running = getRunningSequences();
    NLOG(info) << "Running sequences: " << running.size();
    NLOG(info) << "================================\n";
}

long SequenceController::getStepElapsedTime(std::shared_ptr<Sequence> seq) const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - seq->step_start_time).count();
}

std::string SequenceController::stateToString(SequenceState state) const
{
    switch (state) {
        case SequenceState::STOPPED:
            return "STOPPED";
        case SequenceState::IDLE:
            return "IDLE";
        case SequenceState::RUNNING:
            return "RUNNING";
        case SequenceState::WAITING:
            return "WAITING";
        case SequenceState::COMPLETED:
            return "COMPLETED";
        case SequenceState::ABORTED:
            return "ABORTED";
        case SequenceState::ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

void SequenceController::emergencyStopAllOutputs()
{
    if (!data_handler_)
        return;

    // 모든 출력을 안전 상태로 설정
    data_handler_->setOutputValue(OUTPUT::LIFT, 0x00);
    data_handler_->setOutputValue(OUTPUT::CONVEYOR, 0x00);
    data_handler_->setOutputValue(OUTPUT::LIFT_TURN, 0x00);

    NLOG(info) << "[SAFETY] All outputs set to safe state";
}

void SequenceController::emergencyStopForSequence(std::shared_ptr<Sequence> seq)
{
    // 특정 시퀀스와 관련된 출력만 정지
    // 여기서는 단순히 모든 출력 정지 (실제로는 시퀀스별로 다르게 처리)
    emergencyStopAllOutputs();

    NLOG(info) << "[SAFETY] Emergency stop for sequence: " << seq->name;
}

void SequenceController::onSequenceCompleted(const std::string& sequence_name)
{
    NLOG(info) << "[CALLBACK] Sequence completed: " << sequence_name;

    if(sequence_name == "charge_start")
    {
        std_msgs::Bool charge_state_msg;
        charge_state_msg.data = true;              
        charge_state_pub_.publish(charge_state_msg);

        NLOG(info) << "[CHARGE] Charge Start Sequence completed: " << sequence_name;
    }
    else if(sequence_name == "charge_stop")
    {
        std_msgs::Bool charge_state_msg;
        charge_state_msg.data = false;              
        charge_state_pub_.publish(charge_state_msg);

        NLOG(info) << "[CHARGE] Charge Stop Sequence completed: " << sequence_name;
    }

    // 완료 후 후처리 작업
    // 예: 다음 시퀀스 자동 시작, 로그 기록 등
}

void SequenceController::onSequenceAborted(const std::string& sequence_name, const std::string& reason)
{
    NLOG(info) << "[CALLBACK] Sequence aborted: " << sequence_name << " (" << reason << ")";
    abort_reason_ = reason;  // 저장된 이유를 업데이트

    // 중단 후 후처리 작업
    // 예: 알람 발생, 에러 로그 기록 등
    // lift running time out 에러 발생
}

}  // namespace NaviFra