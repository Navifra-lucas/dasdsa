// nc_sequence_controller.h

#ifndef NC_SEQUENCE_CONTROLLER_H
#define NC_SEQUENCE_CONTROLLER_H

#include <memory>
#include <map>
#include <vector>
#include <string>
#include <functional>
#include <chrono>
#include "nc_io_manager/data/data_handler.h"
#include "nc_io_manager/controller/nc_sequence_controller.h"
#include "core_msgs/NaviAlarm.h"
#include <std_msgs/Int64.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace NaviFra {

// 시퀀스 상태
enum class SequenceState {
    STOPPED,        // 정지 상태
    IDLE,           // 대기 상태
    RUNNING,        // 실행 중태
    WAITING,        // 조건 대기 중
    COMPLETED,      // 완료
    ABORTED,        // 중단
    ERROR           // 오류
};

// 시퀀스 스텝
struct SequenceStep {
    std::string name;
    std::function<void(std::shared_ptr<DataHandler>)> action;     // 실행할 동작
    std::function<bool(std::shared_ptr<DataHandler>)> condition;  // 다음 스텝 조건
    int timeout_ms;                                               // 타임아웃 (ms)
    std::string description;
    
    // 액션 + 조건 생성자
    SequenceStep(const std::string& step_name,
                std::function<void(std::shared_ptr<DataHandler>)> act,
                std::function<bool(std::shared_ptr<DataHandler>)> cond,
                int timeout = 10000,
                const std::string& desc = "")
        : name(step_name), action(act), condition(cond), 
          timeout_ms(timeout), description(desc) {}
    
    // 조건만 있는 생성자 (액션 없음)
    SequenceStep(const std::string& step_name,
                std::function<bool(std::shared_ptr<DataHandler>)> cond,
                int timeout = 10000,
                const std::string& desc = "")
        : name(step_name), action(nullptr), condition(cond), 
          timeout_ms(timeout), description(desc) {}
    
    // 액션만 있는 생성자 (조건은 즉시 true)
    SequenceStep(const std::string& step_name,
                std::function<void(std::shared_ptr<DataHandler>)> act,
                int timeout = 1000,
                const std::string& desc = "")
        : name(step_name), action(act), 
          condition([](std::shared_ptr<DataHandler>) { return true; }),
          timeout_ms(timeout), description(desc) {}
};

// 시퀀스 정의
class Sequence {
public:
    std::string name;
    std::vector<SequenceStep> steps;
    SequenceState state;
    int current_step;
    std::chrono::steady_clock::time_point step_start_time;
    
    Sequence(const std::string& seq_name) 
        : name(seq_name), state(SequenceState::STOPPED), current_step(0) {}
    
    void addStep(const SequenceStep& step) {
        steps.push_back(step);
    }
    
    void reset() {
        state = SequenceState::STOPPED;
        current_step = 0;
    }
};

class SequenceController {
public:
    SequenceController(std::shared_ptr<DataHandler> data_handler, ros::NodeHandle nh);
    virtual ~SequenceController();

private:
    ros::NodeHandle nh_;
    std::shared_ptr<DataHandler> data_handler_;
    std::map<std::string, std::shared_ptr<Sequence>> sequences_;
    ros::Publisher sequence_error_pub_;
    std_msgs::Int64 sequence_error_;

public:
    // 시퀀스 관리
    void addSequence(std::shared_ptr<Sequence> sequence);
    void removeSequence(const std::string& name);
    
    // 시퀀스 실행
    bool startSequence(const std::string& name);
    void stopSequence(const std::string& name);
    void stopAllSequences();

    void SetOutputCommand(const std::string& name);
    
    // 메인 처리 (매 사이클 호출)
    void process();
    
    // 상태 조회
    SequenceState getSequenceState(const std::string& name) const;
    std::vector<std::string> getRunningSequences() const;
    void printStatus() const;
    std::string getAbortReason() const { return abort_reason_; }
    
    // 기본 시퀀스들 설정
    void setupDefaultSequences();

private:
    // 핵심 처리 함수들
    void processSequence(std::shared_ptr<Sequence> seq);
    
    // 상태별 처리 함수들
    void startFirstStep(std::shared_ptr<Sequence> seq);
    void processRunningStep(std::shared_ptr<Sequence> seq);
    void processWaitingStep(std::shared_ptr<Sequence> seq);
    void handleErrorState(std::shared_ptr<Sequence> seq);
    
    // 스텝 실행 관련
    void executeCurrentStep(std::shared_ptr<Sequence> seq);
    bool checkStepCondition(std::shared_ptr<Sequence> seq);
    bool checkStepTimeout(std::shared_ptr<Sequence> seq);
    void advanceToNextStep(std::shared_ptr<Sequence> seq);
    void completeSequence(std::shared_ptr<Sequence> seq);
    void abortSequence(std::shared_ptr<Sequence> seq, const std::string& reason);
    
    // 유틸리티 함수들
    long getStepElapsedTime(std::shared_ptr<Sequence> seq) const;
    std::string stateToString(SequenceState state) const;
    
    // 안전 관련
    void emergencyStopAllOutputs();
    void emergencyStopForSequence(std::shared_ptr<Sequence> seq);
    
    // 콜백 함수들
    void onSequenceCompleted(const std::string& sequence_name);
    void onSequenceAborted(const std::string& sequence_name, const std::string& reason);

    // 시퀀스 상태 데이터
    std::string abort_reason_;

    ros::Publisher charge_state_pub_;
    std::string s_outputcommand_;
};

} // namespace NaviFra

#endif // NC_SEQUENCE_CONTROLLER_H