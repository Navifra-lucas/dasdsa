#include "nc_cheonil_agent/manager/nc_cheonil_manager.h"

#include "nc_cheonil_agent/nc_cheonil_agent_pch.h"

using namespace NaviFra;

NcCheonilManager& NcCheonilManager::instance()
{
    static Poco::SingletonHolder<NcCheonilManager> sh;
    return *sh.get();
}

NcCheonilManager::NcCheonilManager()
    : bRunning_(false)
{
}

NcCheonilManager::~NcCheonilManager()
{
    bRunning_ = false;
    worker_.join();
}

bool NcCheonilManager::initialize(ros::NodeHandle& nh)
{
    if (NcCheonilPDUManager::instance().initialize(nh) != true) {
        NLOG(error) << "NcCheonilPDUManager initialize fail";
        return false;
    }

    bRunning_ = true;
    worker_.start(*this);

    return true;
}

void NcCheonilManager::run()
{
    int16_t n_Pre_CommandNum = -1;
    NLOG(debug) << "NcCheonilManager::run() started" << std::endl;
    while (bRunning_) {
        NLOG(debug) << "NcCheonilManager::run() - while loop iteration start" << std::endl;
        
        NLOG(debug) << "Reading COMMAND_NUM register..." << std::endl;
        int16_t n_CommandNum = NcCheonilPDUManager::instance().readRegister(PDUdefinition::PDU_READ_REGISTER_COMMAND_NUM);
        NLOG(debug) << "COMMAND_NUM read: " << n_CommandNum << ", previous: " << n_Pre_CommandNum << std::endl;

        if (n_CommandNum != n_Pre_CommandNum) {
            NLOG(debug) << "Command changed, calling handleActionPLC()..." << std::endl;
            actionHandler_.handleActionPLC();
            NLOG(debug) << "handleActionPLC() completed" << std::endl;
        }

        NLOG(debug) << "Calling handleStatusPLC()..." << std::endl;
        statusHandler_.handleStatusPLC();
        NLOG(debug) << "handleStatusPLC() completed" << std::endl;
        
        NLOG(debug) << "Calling Status()..." << std::endl;
        statusHandler_.Status();
        NLOG(debug) << "Status() completed" << std::endl;
        
        // statusHandler_.ObstacleCheck();
        
        NLOG(debug) << "Calling JogMove()..." << std::endl;
        statusHandler_.JogMove();
        NLOG(debug) << "JogMove() completed" << std::endl;
        
        // statusHandler_.JogLeftRight();
        
        NLOG(debug) << "Calling Charger()..." << std::endl;
        statusHandler_.Charger();
        NLOG(debug) << "Charger() completed" << std::endl;
        
        NLOG(debug) << "Calling ManualCharger()..." << std::endl;
        // statusHaqndler_.ManualCharger();
        NLOG(debug) << "ManualCharger() completed" << std::endl;
        
        // statusHandler_.InPolygon();

        n_Pre_CommandNum = n_CommandNum;

        NLOG(debug) << "Before sleep, bRunning_: " << (bRunning_ ? "true" : "false") << std::endl;
        NLOG(debug) << "Sleeping for 50ms..." << std::endl;
        Poco::Thread::sleep(50);
        NLOG(debug) << "Sleep completed, bRunning_: " << (bRunning_ ? "true" : "false") << std::endl;
        NLOG(debug) << "NcCheonilManager::run() - while loop iteration end" << std::endl;
    }
    NLOG(debug) << "NcCheonilManager::run() exited" << std::endl;
}
