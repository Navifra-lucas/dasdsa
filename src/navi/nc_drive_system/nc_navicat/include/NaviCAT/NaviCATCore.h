#ifndef NAVIFRA_NAVICAT_CORE_H
#define NAVIFRA_NAVICAT_CORE_H

#include "NaviCAT/interface/IMotor.h"
#include "NaviCAT/Canopen.h"
#include "NaviCAT/PDO.h"

#include <memory>
#include <iostream>
#include <atomic>
#include <array>

#include "soem/soem.h"

namespace NaviFra {
namespace NaviCAT {

using namespace NaviFra::NaviCAT::Canopen;

class NaviCATCore : public IMotor {

public:
    static constexpr int EC_TIMEOUTMON = 500;
    static constexpr int SLAVE_NUM = 1;

    using PDOMapper = NaviFra::NaviCAT::PDO::PDOMapper;
    using PDOData = NaviFra::NaviCAT::PDO::PDOData;
    using PDOEntry = NaviFra::NaviCAT::PDO::PDOEntry;

    NaviCATCore() = default;
    ~NaviCATCore() = default;

    bool initialize();
    bool finalize();

    bool initialize(const std::string& interface, const std::string& yaml_file);
    bool initializeMotor();
    void runJogControl();
    void cleanup();

    int getState(uint8_t motorId) override {return 0; }
    bool isEnable(uint8_t motorId) override {return false;}
    bool isError(uint8_t motorId) override {return false;}

    void setTarget(uint8_t motorId, double value) override {}
    double getPosition(uint8_t motorId) override { return 0; }
    double getSpeed(uint8_t motorId) override { return 0; }
    double getWheelRPM(uint8_t motorId) override { return 0; }
    double getMotorRPM(uint8_t motorId) override { return 0; }
    double getCurrent(uint8_t motorId) override { return 0; }
    double getVoltage(uint8_t motorId) override { return 0; }
    uint32_t getErrorCode(uint8_t motorId) override { return 0; }
    std::string getErrorMessage(uint8_t motorId) override { return ""; }
    uint16_t getStatus(uint8_t motorId) override { return 0; }

    bool enable(uint8_t motorId) override {return false;}
    bool disable(uint8_t motorId) override {return false;}
    bool resetError(uint8_t motorId) override {return false;}

    double getLatestRpdoWriteTimeCount(uint8_t motorId) const override {return false;}
    int getCanBusState() override {return 0;}

    bool kbhit();

    void motorStateMachine();
    void printAdapters();
private:

    std::unique_ptr<PDOMapper> pdo_mapper_;
    std::unique_ptr<PDOData> outputs_;
    std::unique_ptr<PDOData> inputs_;
    
    std::array<char, 4096> IOmap_;
    std::atomic<int> wkc_{0};
    std::atomic<bool> inOP_{false};
    std::atomic<bool> running_{false};

    ecx_context ecx_context_;
    ec_adapter* adapter_list_;
};

}
}

#endif 