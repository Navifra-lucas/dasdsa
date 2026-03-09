#ifndef NC_CHEONIL_PDU_MANAGER_H
#define NC_CHEONIL_PDU_MANAGER_H

#include "nc_cheonil_agent/nc_cheonil_agent.h"

#include <Poco/Thread.h>
#include <core_agent/modbus/modbus_controller.h>
#include <core_msgs/CheonilReadRegister.h>
#include <nc_cheonil_agent/data/PDU.h>
#include <ros/node_handle.h>

#include <memory>

namespace NaviFra {
class NcCheonilPDUManager : public Poco::Runnable {
public:
    NcCheonilPDUManager();
    ~NcCheonilPDUManager();

public:
    virtual void run();
    bool initialize(ros::NodeHandle& nh);
    static NcCheonilPDUManager& instance();

public:
    bool readCoil(std::string key);
    int16_t readRegister(std::string key);

    bool writeRegister(std::string key, int16_t value);
    bool writeCoil(std::string key, bool value);
    void fillMsg(core_msgs::CheonilReadRegister& msg) const;

private:
    boost::asio::io_context io_context_;
    std::unique_ptr<ModbusController> modbusController_;

    PDU address_map_manager_;

    std::vector<uint16_t> write_register_;
    std::vector<int16_t> read_register_;

    std::vector<bool> write_coil_;
    std::vector<bool> read_coil_;

    std::atomic<bool> bRunning_;
    Poco::Thread worker_;

    Poco::FastMutex fastMutex_;

    bool b_is_emul_;
    ros::Publisher read_register_pub_;
};
}  // namespace NaviFra

#endif  // NC_CHEONIL_PDU_MANAGER_H