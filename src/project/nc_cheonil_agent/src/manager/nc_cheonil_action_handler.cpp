#include "nc_cheonil_agent/manager/nc_cheonil_action_handler.h"

using namespace NaviFra;

NcCheonilActionHandler::NcCheonilActionHandler()
{
}

NcCheonilActionHandler::~NcCheonilActionHandler()
{
}

void NcCheonilActionHandler::handleActionPLC()
{
    try {
        NLOG(debug) << "handleActionPLC() - start" << std::endl;
        NLOG(debug) << "handleActionPLC() - calling ReadRegisterData()..." << std::endl;
        ReadRegisterData();
        NLOG(debug) << "handleActionPLC() - ReadRegisterData() completed" << std::endl;
        // ReadCoilData();
        // LeftLoad();
        // LeftUnLoad();
        // RightLoad();
        // RightUnLoad();
        // ChargingStart();
        // ChargingEnd();
        // Reset();
        // AlarmReset();
        // ConveyorCW();
        // ConveyorCCW();
        // ConveyorStop();
        // LeftStopperUp();
        // LeftStopperDown();
        // RightStopperUp();
        // RightStopperDown();
        NLOG(debug) << "handleActionPLC() - end" << std::endl;
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "handleActionPLC response failed: " << e.displayText();
    }
    catch (const std::exception& e) {
        NLOG(error) << "handleActionPLC std::exception: " << e.what();
    }
    catch (...) {
        NLOG(error) << "handleActionPLC unknown exception";
    }
}

void NcCheonilActionHandler::ReadRegisterData()
{
    try {
        NLOG(debug) << "ReadRegisterData() - start" << std::endl;
        auto& pdu = NcCheonilPDUManager::instance();

        ReadRegister read_register;

        NLOG(debug) << "ReadRegisterData() - reading all registers..." << std::endl;
        read_register.command_num               = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_COMMAND_NUM);
        read_register.command                   = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_COMMAND);
        read_register.target_node               = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_TARGET_NODE);
        read_register.x_position                = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_X_POSITION);
        read_register.y_position                = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_Y_POSITION);
        read_register.angle_position            = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_ANGLE_POSITION);
        read_register.speed_limit               = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_SPEED_LIMIT);
        read_register.jog_enable                = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOG_ENABLE);
        read_register.jog_speed                 = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOG_SPEED);
        read_register.jog_angle_speed           = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOG_ANGLE_SPEED);
        read_register.jog_steer_deg             = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOG_STEER_DEG);
        read_register.speed_type                = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_SPEED_TYPE);
        read_register.none_speed                = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_NONE_SPEED);
        read_register.empty_speed               = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_EMPTY_SPEED);
        read_register.load_state                = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_LOAD_STATE);
        read_register.auto_on                   = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_AUTO_ON);
        read_register.battery                   = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_BATTERY);
        read_register.fork_up_down_position     = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_FORK_UP_DOWN_POSITION);
        read_register.fork_up_down_complete     = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_FORK_UP_DOWN_COMPLETE);
        read_register.tilting_up_down           = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_TILTING_UP_DOWN);
        read_register.fork_width                = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_FORK_WIDTH);
        read_register.pallet_touch              = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_PALLET_TOUCH);
        read_register.lsc_1                     = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_LSC_1);
        read_register.lsc_2                     = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_LSC_2);
        read_register.lsc_3                     = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_LSC_3);
        read_register.lsc_4                     = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_LSC_4);
        read_register.charge_state              = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_CHARGE_STATE);
        read_register.job_cancel                = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOB_CANCEL);
        read_register.pallet_id_remove          = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_PALLET_ID_REMOVE);
        read_register.plc_alarm                 = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_PLC_ALARM);
        NLOG(debug) << "ReadRegisterData() - all registers read" << std::endl;

        NLOG(debug) << "ReadRegisterData() - calling CheonilReadRegister()..." << std::endl;
        CheonilReadRegister(read_register);
        NLOG(debug) << "ReadRegisterData() - CheonilReadRegister() completed" << std::endl;
        NLOG(debug) << "ReadRegisterData() - end" << std::endl;
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "ReadRegisterData response failed: " << e.displayText();
    }
    catch (const std::exception& e) {
        NLOG(error) << "ReadRegisterData std::exception: " << e.what();
    }
    catch (...) {
        NLOG(error) << "ReadRegisterData unknown exception";
    }
}

void NcCheonilActionHandler::ReadCoilData()
{
    try {
        auto& pdu = NcCheonilPDUManager::instance();

        ReadCoil read_coil;

        read_coil.read_front_safety_scanner = pdu.readCoil(PDUdefinition::PDU_READ_COIL_FRONT_SAFETY_SCANNER);
        read_coil.read_left_safety_scanner = pdu.readCoil(PDUdefinition::PDU_READ_COIL_LEFT_SAFETY_SCANNER);
        read_coil.read_right_safety_scanner = pdu.readCoil(PDUdefinition::PDU_READ_COIL_RIGHT_SAFETY_SCANNER);

        CheonilReadCoil(read_coil);
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "ReadCoilData response failed: " << e.displayText();
    }
}



void NcCheonilActionHandler::LeftLoad()
{
    try {

    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "LeftLoad response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::LeftUnLoad()
{
    try {

    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "LeftUnLoad response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::RightLoad()
{
    try {

    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "RightLoad response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::RightUnLoad()
{
    try {

    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "RightUnLoad response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::ChargingStart()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "ChargingStart response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::ChargingEnd()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "ChargingEnd response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::Reset()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "Reset response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::AlarmReset()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "AlarmReset response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::ConveyorCW()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "ConveyorCW response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::ConveyorCCW()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "ConveyorCCW response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::ConveyorStop()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "ConveyorStop response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::LeftStopperUp()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "LeftStopperUp response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::LeftStopperDown()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "LeftStopperDown response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::RightStopperUp()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "RightStopperUp response failed: " << e.displayText();
    }
}

void NcCheonilActionHandler::RightStopperDown()
{
    try {

        
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "RightStopperDown response failed: " << e.displayText();
    }
}