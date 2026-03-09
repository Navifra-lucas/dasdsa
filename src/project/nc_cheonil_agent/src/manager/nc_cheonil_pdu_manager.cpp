#include "nc_cheonil_agent/nc_cheonil_agent_pch.h"

#include <Poco/SingletonHolder.h>
#include <nc_cheonil_agent/manager/nc_cheonil_pdu_manager.h>

using namespace NaviFra;

NcCheonilPDUManager& NcCheonilPDUManager::instance()
{
    static Poco::SingletonHolder<NcCheonilPDUManager> sh;
    return *sh.get();
}
NcCheonilPDUManager::NcCheonilPDUManager()
    : b_is_emul_(false)
{
}

NcCheonilPDUManager::~NcCheonilPDUManager()
{
    bRunning_ = false;
    worker_.join();
}

void NcCheonilPDUManager::run()
{
    NLOG(debug) << "PDUManager::run() started" << std::endl;
    while (bRunning_) {
        NLOG(debug) << "PDUManager::run() - while loop iteration start, bRunning_: " << (bRunning_ ? "true" : "false") << std::endl;
        if (b_is_emul_) {
            NLOG(debug) << "PDUManager::run() - emulation mode" << std::endl;
            {
                NLOG(debug) << "PDUManager::run() - acquiring mutex for emul..." << std::endl;
                Poco::FastMutex::ScopedLock lock(fastMutex_);
                NLOG(debug) << "PDUManager::run() - mutex locked for emul" << std::endl;

                core_msgs::CheonilReadRegister msg;
                fillMsg(msg);
                read_register_pub_.publish(msg);
                NLOG(debug) << "PDUManager::run() - emul message published, releasing mutex..." << std::endl;
            }
            NLOG(debug) << "PDUManager::run() - mutex released for emul, sleeping..." << std::endl;
            Poco::Thread::sleep(50);
            NLOG(debug) << "PDUManager::run() - emul sleep completed" << std::endl;
            continue;
        }
        try {
            NLOG(debug) << "PDUManager::run() - calling ReadInputRegisters..." << std::endl;
            auto read_register = modbusController_->ReadInputRegisters(
                0x00, address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_START),
                address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_END) + 1 -
                    address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_START));
            NLOG(debug) << "PDUManager::run() - ReadInputRegisters completed" << std::endl;

            {
                NLOG(debug) << "PDUManager::run() - acquiring mutex to update read_register_..." << std::endl;
                Poco::FastMutex::ScopedLock lock(fastMutex_);
                NLOG(debug) << "PDUManager::run() - mutex locked, updating read_register_..." << std::endl;
                read_register_.assign(read_register.begin(), read_register.end());
                NLOG(debug) << "PDUManager::run() - read_register_ updated, releasing mutex..." << std::endl;
            }
            NLOG(debug) << "PDUManager::run() - mutex released after read_register_ update" << std::endl;

            // // auto read_heart_beat = readRegister(PDUdefinition::PDU_READ_REGISTER_HEARTBEAT);

            // Copy write_register_ to local variable before Modbus call (outside mutex)
            std::vector<uint16_t> write_register_copy;
            {
                NLOG(debug) << "PDUManager::run() - acquiring mutex to copy write_register_..." << std::endl;
                Poco::FastMutex::ScopedLock lock(fastMutex_);
                NLOG(debug) << "PDUManager::run() - mutex locked, copying write_register_..." << std::endl;
                write_register_copy = write_register_;
                NLOG(debug) << "PDUManager::run() - write_register_ copied, releasing mutex..." << std::endl;
            }
            NLOG(debug) << "PDUManager::run() - mutex released, calling WriteMultipleRegisters (outside mutex)..." << std::endl;
            modbusController_->WriteMultipleRegisters(
                0x00, address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_START), write_register_copy);
            NLOG(debug) << "PDUManager::run() - WriteMultipleRegisters completed" << std::endl;

            // Copy write_coil_ to local variable before Modbus call (outside mutex)
            std::vector<bool> write_coil_copy;
            {
                NLOG(debug) << "PDUManager::run() - acquiring mutex to copy write_coil_..." << std::endl;
                Poco::FastMutex::ScopedLock lock(fastMutex_);
                NLOG(debug) << "PDUManager::run() - mutex locked, copying write_coil_..." << std::endl;
                write_coil_copy = write_coil_;
                NLOG(debug) << "PDUManager::run() - write_coil_ copied, releasing mutex..." << std::endl;
            }
            NLOG(debug) << "PDUManager::run() - mutex released, calling WriteMultipleCoils (outside mutex)..." << std::endl;
        }
        catch (const std::exception& ex) {
            NLOG(error) << "[DEBUG] PDUManager::run() - exception caught: " << ex.what() << std::endl;
        }
        catch (...) {
            NLOG(error) << "[DEBUG] PDUManager::run() - unknown exception caught" << std::endl;
        }

        NLOG(debug) << "PDUManager::run() - before sleep, bRunning_: " << (bRunning_ ? "true" : "false") << std::endl;
        Poco::Thread::sleep(50);
        NLOG(debug) << "PDUManager::run() - sleep completed, bRunning_: " << (bRunning_ ? "true" : "false") << std::endl;
        NLOG(debug) << "PDUManager::run() - while loop iteration end" << std::endl;
    }
    NLOG(debug) << "PDUManager::run() exited" << std::endl;
}

bool NcCheonilPDUManager::initialize(ros::NodeHandle& nh)
{
    read_register_.resize(
        address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_END) -
            address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_START),
        0);
    write_register_.resize(
        address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_END) -
            address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_START),
        0);

    write_coil_.resize(
        address_map_manager_.getCoil(PDUdefinition::PDU_WRITE_COIL_END) + 1 -
            address_map_manager_.getCoil(PDUdefinition::PDU_WRITE_COIL_START),
        false);
    read_coil_.resize(
        address_map_manager_.getCoil(PDUdefinition::PDU_READ_COIL_END) + 1 -
            address_map_manager_.getCoil(PDUdefinition::PDU_READ_COIL_START),
        false);

    std::string portType;
    nh.param<bool>("is_emul", b_is_emul_, false);
    nh.param<std::string>("cheonilagent/modbus/type", portType, "tcp");

    read_register_pub_ = nh.advertise<core_msgs::CheonilReadRegister>("cheonil/read_register", 1);

    LOG_INFO("Modbus port type: %s", portType.c_str());

    if (b_is_emul_) {
        LOG_INFO("Start Emulation Mode (Bypass)");
        // CheonilEmulManager::instance().initialize();
    }
    else {
        try {
            if (portType == "rtu") {
                std::string device;
                int baudRate;
                nh.param<std::string>("cheonilagent/modbus/rtu/device", device, "/dev/ttyUSB0");
                nh.param<int>("cheonilagent/modbus/baudRate", baudRate, 115200);

                LOG_INFO("RTU device: %s", device.c_str());
                LOG_INFO("RTU baud rate: %d", baudRate);
                modbusController_.reset(new ModbusController(io_context_, ModbusController::PortType::RTU, "/dev/ttyV1", "9600"));

                LOG_INFO("RTU port initialized.");
            }
            else {
                std::string host;
                std::string port_number;

                const char* env_host = std::getenv("MODBUS_HOST");
                if (env_host != nullptr) {
                    host = std::string(env_host);
                }
                else {
                    host = "192.168.0.20";
                }

                const char* env_port = std::getenv("MODBUS_PORT");
                if (env_port != nullptr) {
                    port_number = env_port;
                }
                else {
                    port_number = "502";
                }

                NLOG(info) << "TCP host: " << host;
                NLOG(info) << "TCP port: " << port_number;

                modbusController_.reset(new ModbusController(io_context_, ModbusController::PortType::TCP, host, port_number));

                LOG_INFO("TCP port initialized.");
            }
        }
        catch (Poco::Exception& ex) {
            LOG_INFO("Exception during initialization: %s", ex.what());
            return false;
        }
    }

    bRunning_ = true;
    worker_.start(*this);

    return true;
}

bool NcCheonilPDUManager::writeRegister(std::string key, int16_t value)
{
    NLOG(debug) << "writeRegister() - start, key: " << key << ", value: " << value << std::endl;
    auto address = address_map_manager_.getHoldingRegister(key);
    auto writeStartAddr = address_map_manager_.getHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_START);
    address = address - writeStartAddr;
    if (write_register_.size() < address) {
        NLOG(error) << "[DEBUG] writeRegister() - address out of range: " << address << ", size: " << write_register_.size() << std::endl;
        return false;
    }
    NLOG(debug) << "writeRegister() - acquiring mutex lock..." << std::endl;
    {
        Poco::FastMutex::ScopedLock lock(fastMutex_);
        NLOG(debug) << "writeRegister() - mutex locked, writing value..." << std::endl;
        write_register_[address] = value;
        NLOG(debug) << "writeRegister() - value written, releasing mutex..." << std::endl;
    }
    NLOG(debug) << "writeRegister() - mutex released, end" << std::endl;
    return true;
}
bool NcCheonilPDUManager::writeCoil(std::string key, bool value)
{
    auto address = address_map_manager_.getCoil(key);
    auto writeStartAddr = address_map_manager_.getCoil(PDUdefinition::PDU_WRITE_COIL_START);
    address = address - writeStartAddr;
    if (write_coil_.size() < address) {
        return false;
    }
    {
        Poco::FastMutex::ScopedLock lock(fastMutex_);
        write_coil_[address] = value;
    }
    return true;
}
int16_t NcCheonilPDUManager::readRegister(std::string key)
{
    NLOG(debug) << "readRegister() - start, key: " << key << std::endl;
    auto address = address_map_manager_.getHoldingRegister(key);
    auto writeStartAddr = address_map_manager_.getHoldingRegister(PDUdefinition::PDU_READ_REGISTER_START);
    address = address - writeStartAddr;
    if (read_register_.size() <= address) {
        NLOG(error) << "[DEBUG] readRegister() - address out of range: " << address << ", size: " << read_register_.size() << std::endl;
        return 0;
    }
    int16_t value = 0;
    NLOG(debug) << "readRegister() - acquiring mutex lock..." << std::endl;
    {
        Poco::FastMutex::ScopedLock lock(fastMutex_);
        NLOG(debug) << "readRegister() - mutex locked, reading value..." << std::endl;
        value = read_register_[address];
        NLOG(debug) << "readRegister() - value read: " << value << ", releasing mutex..." << std::endl;
    }
    NLOG(debug) << "readRegister() - mutex released, end, returning: " << value << std::endl;
    return value;
}

bool NcCheonilPDUManager::readCoil(std::string key)
{
    auto address = address_map_manager_.getCoil(key);
    auto writeStartAddr = address_map_manager_.getCoil(PDUdefinition::PDU_READ_COIL_START);
    address = address - writeStartAddr;
    if (read_coil_.size() <= address) {
        return 0;
    }
    bool value = false;
    {
        Poco::FastMutex::ScopedLock lock(fastMutex_);
        value = read_coil_[address];
    }
    return value;
}

void NcCheonilPDUManager::fillMsg(core_msgs::CheonilReadRegister& msg) const
{
    msg.command_num = read_register_[0];
    msg.command = read_register_[1];
    msg.target_node = read_register_[2];
    msg.x_position = read_register_[3];
    msg.y_position = read_register_[4];
    msg.angle_position = read_register_[5];
    msg.speed_limit = read_register_[6];
    msg.jog_enable = read_register_[7];
    msg.jog_speed = read_register_[8];
    msg.jog_angle_speed = read_register_[9];
    msg.jog_steer_deg = read_register_[10];
    msg.speed_type = read_register_[11];
    msg.none_speed = read_register_[12];
    msg.empty_speed = read_register_[13];
    msg.load_state = read_register_[14];
    msg.auto_on = read_register_[15];
    msg.battery = read_register_[16];
    msg.fork_up_down_position = read_register_[17];
    msg.fork_up_down_complete = read_register_[18];
    msg.tilting_up_down = read_register_[19];
    msg.fork_width = read_register_[20];
    msg.lsc_1 = read_register_[21];
    msg.lsc_2 = read_register_[22];
    msg.lsc_3 = read_register_[23];
    msg.lsc_4 = read_register_[24];
    msg.pallet_touch = read_register_[25];
    msg.charge_state = read_register_[26];
    msg.job_cancel = read_register_[27];
    msg.pallet_id_remove = read_register_[28];
    msg.plc_alarm = read_register_[29];
}
