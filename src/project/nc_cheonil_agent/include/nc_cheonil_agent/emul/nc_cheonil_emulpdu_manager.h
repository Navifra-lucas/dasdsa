// #ifndef NC_CHEONIL_EMUL_PDU_MANAGER_H
// #define NC_CHEONIL_EMUL_PDU_MANAGER_H

// #include <Poco/Thread.h>
// #include <core_agent/modbus/modbus_controller.h>
// #include <nc_cheonil_agent/data/PDU.h>
// #include <ros/ros.h>
// #include <ros/node_handle.h>

// #include <memory>

// namespace NaviFra {
// class NcCheonilEmulPDUManager : public Poco::Runnable {
// public:
//     NcCheonilEmulPDUManager();
//     ~NcCheonilEmulPDUManager();

// public:
//     virtual void run();
//     bool initialize(ros::NodeHandle& nh);
//     static NcCheonilEmulPDUManager& instance();

// public:
//     bool readCoil(std::string key);
//     int16_t readRegister(std::string key);

//     bool writeRegister(std::string key, int16_t value);
//     bool writeCoil(std::string key, bool value);

// private:
//     boost::asio::io_context io_context_;
//     std::unique_ptr<ModbusController> modbusController_;

//     PDU address_map_manager_;

//     std::vector<uint16_t> write_register_;
//     std::vector<uint16_t> read_register_;

//     std::vector<bool> write_coil_;
//     std::vector<bool> read_coil_;

//     std::atomic<bool> bRunning_;
//     Poco::Thread worker_;

//     Poco::FastMutex fastMutex_;
// };
// }  // namespace NaviFra

// #endif  // NC_CHEONIL_PDU_MANAGER_H
