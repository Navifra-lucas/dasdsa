// #include "nc_cheonil_agent/nc_cheonil_agent_pch.h"

// #include <Poco/SingletonHolder.h>
// #include <nc_cheonil_agent/emul/nc_cheonil_emulpdu_manager.h>
// #include <nc_cheonil_agent/emul/nc_cheonil_Emul_Manager.h>

// using namespace NaviFra;

// NcCheonilEmulPDUManager& NcCheonilEmulPDUManager::instance()
// {
//     static Poco::SingletonHolder<NcCheonilEmulPDUManager> sh;
//     return *sh.get();
// }
// NcCheonilEmulPDUManager::NcCheonilEmulPDUManager()
// {
// }

// NcCheonilEmulPDUManager::~NcCheonilEmulPDUManager()
// {
//     bRunning_ = false;
//     worker_.join();
// }

// void NcCheonilEmulPDUManager::run()
// {
//     uint heart_beat = 0;
//     int target_level = 0;
//     while (bRunning_) {
//         try {
//             writeRegister(PDUdefinition::PDU_READ_REGISTER_HEARTBEAT, heart_beat);

//             {
//                 Poco::FastMutex::ScopedLock lock(fastMutex_);
//                 modbusController_->WriteMultipleRegisters(
//                     0x00, address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_START), read_register_);
//             }

//             auto read_register = modbusController_->ReadHoldingRegisters(
//                 0x00, address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_START),
//                 address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_END) -
//                     address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_START));
//             {
//                 Poco::FastMutex::ScopedLock lock(fastMutex_);
//                 write_register_.assign(read_register.begin(), read_register.end());
//             }

//             {
//                 auto write_coil =modbusController_->ReadCoils(0x00, address_map_manager_.getCoil(PDUdefinition::PDU_WRITE_COIL_START),
//                 address_map_manager_.getCoil(PDUdefinition::PDU_WRITE_COIL_END) - 
//                 address_map_manager_.getCoil(PDUdefinition::PDU_WRITE_COIL_START));

//                 Poco::FastMutex::ScopedLock lock(fastMutex_);
//                 write_coil_.assign(write_coil.begin(), write_coil.end());
//             }

//             {
//                 modbusController_->WriteMultipleCoils(0x00, address_map_manager_.getCoil(PDUdefinition::PDU_READ_COIL_START), read_coil_);
//             }

//             heart_beat++;
//             if(heart_beat == 256) heart_beat = 0; 
//             // NLOG(info) << "Heartbeat PLC: " << heart_beat;
//             // NLOG(info) << "Current Level PLC: %d " << target_level;
//         }
//         catch (const std::exception& ex) {
//         }

//         Poco::Thread::sleep(50);
//     }
// }

// bool NcCheonilEmulPDUManager::initialize(ros::NodeHandle& nh)
// {
//     write_register_.resize(
//         address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_END) -
//             address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_START),
//         0);
//     read_register_.resize(
//         address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_END) -
//             address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_START),
//         0);

//     write_coil_.resize(
//         address_map_manager_.getCoil(PDUdefinition::PDU_WRITE_COIL_END) - address_map_manager_.getCoil(PDUdefinition::PDU_WRITE_COIL_START), false);
//     read_coil_.resize(address_map_manager_.getCoil(PDUdefinition::PDU_READ_COIL_END) - address_map_manager_.getCoil(PDUdefinition::PDU_READ_COIL_START), false);

//     std::string portType;
//     nh.param<std::string>("cheonilagent/modbus/type", portType, "tcp");

//     LOG_INFO("Modbus port type: %s", portType.c_str());

//     try {
//         if (portType == "rtu") {
//             std::string device;
//             int baudRate;
//             nh.param<std::string>("cheonilagent/modbus/rtu/device", device, "/dev/ttyUSB0");
//             nh.param<int>("cheonilagent/modbus/baudRate", baudRate, 115200);

//             LOG_INFO("RTU device: %s", device.c_str());
//             LOG_INFO("RTU baud rate: %d", baudRate);
//             modbusController_.reset(new ModbusController(io_context_, ModbusController::PortType::RTU, "/dev/ttyV1", "9600"));

//             LOG_INFO("RTU port initialized.");
//         }
//         else {
//             // std::string host;
//             // int port_number;

//             // LOG_INFO("TCP host: %s", host.c_str());
//             // LOG_INFO("TCP port: %d", port_number);

//             std::string host;
//             std::string port_number;

//             nh.param<std::string>("plc_host", host, "127.0.0.1");
//             nh.param<std::string>("plc_port", port_number, "502");


//             NLOG(info) << "TCP host: " << host;
//             NLOG(info) << "TCP port: " << port_number;

//             modbusController_.reset(new ModbusController(io_context_, ModbusController::PortType::TCP, host, port_number));
//             // modbusController_.reset(new ModbusController(io_context_, ModbusController::PortType::TCP, "127.0.0.1", "502"));

//             LOG_INFO("TCP port initialized.");
//         }
//     }
//     catch (Poco::Exception& ex) {
//         LOG_INFO("Exception during initialization: %s", ex.what());
//         return false;
//     }

//     CheonilEmulManager::instance().initialize();
                                    

//     bRunning_ = true;
//     worker_.start(*this);

//     return true;
// }

// bool NcCheonilEmulPDUManager::writeRegister(std::string key, int16_t value)
// {
//     auto address = address_map_manager_.getHoldingRegister(key);
//     auto writeStartAddr = address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_START);
//     address = address - writeStartAddr;
//     if (read_register_.size() < address) {
//         return false;
//     }
//     {
//         Poco::FastMutex::ScopedLock lock(fastMutex_);
//         read_register_[address] = value;
//     }
//     return true;
// }
// bool NcCheonilEmulPDUManager::writeCoil(std::string key, bool value)
// {
//     auto address = address_map_manager_.getCoil(key);
//     auto writeStartAddr = address_map_manager_.getCoil(PDUdefinition::PDU_READ_COIL_START);
//     address = address - writeStartAddr;
//     if (read_coil_.size() < address) {
//         return false;
//     }
//     {
//         Poco::FastMutex::ScopedLock lock(fastMutex_);
//         read_coil_[address] = value;
//     }
//     return true;
// }
// int16_t NcCheonilEmulPDUManager::readRegister(std::string key)
// {
//     auto address = address_map_manager_.getHoldingRegister(key);
//     auto writeStartAddr = address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_START);
//     address = address - writeStartAddr;
//     if (write_register_.size() <= address) {
//         return 0;
//     }
//     int16_t value = 0;
//     {
//         Poco::FastMutex::ScopedLock lock(fastMutex_);
//         value = write_register_[address];
//     }
//     return value;
// }

// bool NcCheonilEmulPDUManager::readCoil(std::string key)
// {
//     auto address = address_map_manager_.getCoil(key);
//     auto writeStartAddr = address_map_manager_.getCoil(PDUdefinition::PDU_WRITE_COIL_START);
//     address = address - writeStartAddr;
//     if (write_coil_.size() <= address) {
//         return 0;
//     }
//     bool value = false;
//     {
//         Poco::FastMutex::ScopedLock lock(fastMutex_);
//         value = write_coil_[address];
//     }
//     return value;
// }