#ifndef NC_CHEONIL_MANAGER_H
#define NC_CHEONIL_MANAGER_H

#include <Poco/Runnable.h>
#include <Poco/Thread.h>
#include <Poco/SingletonHolder.h>

#include <mutex>

#include "nc_cheonil_agent/state/CheonilState.h"
#include "nc_cheonil_agent/emul/nc_cheonil_emulpdu_manager.h"
#include "nc_cheonil_agent/manager/nc_cheonil_pdu_manager.h"
#include "nc_cheonil_agent/data/PDU.h"
#include "nc_cheonil_agent/data/PDUdefinition.h"
#include "nc_cheonil_agent/manager/nc_cheonil_action_handler.h"
#include "nc_cheonil_agent/manager/nc_cheonil_status_handler.h"

namespace NaviFra {
class NcCheonilManager : public Poco::Runnable {
public:
    NcCheonilManager();
    ~NcCheonilManager();

    virtual void run();

    bool initialize(ros::NodeHandle& nh);
    static NcCheonilManager& instance();

private:
    NcCheonilActionHandler actionHandler_;
    NcCheonilStatusHandler statusHandler_;

    // CheonilController cheonilController_;
    ros::Subscriber sub_cheonil_;
    std::string s_cheonil_mode;
    int n_cheonil_level;
    bool b_cheonil_action;
    std::mutex data_mutex; 


    std::atomic<bool> bRunning_;
    Poco::Thread worker_;
};
}  // namespace NaviFra

#endif  // NC_CHEONIL_MANAGER_H