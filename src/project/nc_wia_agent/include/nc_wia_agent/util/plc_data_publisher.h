#pragma once

#include <mutex>
#include <string>

namespace NaviFra {

class PlcTaskPublisher {
public:
    static PlcTaskPublisher& instance();

    // PLC 메시지 발행
    void publishOffset(std::string name, double value);
    void controlScenario(bool b_action_type, int n_scenario_index);

private:
    PlcTaskPublisher();
    ~PlcTaskPublisher();
    PlcTaskPublisher(const PlcTaskPublisher&) = delete;
    void operator=(const PlcTaskPublisher&) = delete;

    std::string producer_;
    std::string plc_;
    std::mutex mtx_;
};

}  // namespace NaviFra
