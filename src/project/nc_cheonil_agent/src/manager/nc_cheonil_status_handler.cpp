#include "nc_cheonil_agent/manager/nc_cheonil_status_handler.h"

using namespace NaviFra;

NcCheonilStatusHandler::NcCheonilStatusHandler()
{
    auto librobotInfo = InMemoryRepository::instance().get<NaviFra::RobotInfo>(NaviFra::RobotInfo::KEY);
}

NcCheonilStatusHandler::~NcCheonilStatusHandler()
{
}

void NcCheonilStatusHandler::handleStatusPLC()
{
    try {
        NLOG(debug) << "handleStatusPLC() - start" << std::endl;

        NLOG(debug) << "Getting RobotInfo from InMemoryRepository..." << std::endl;
        auto librobotInfo = InMemoryRepository::instance().get<NaviFra::RobotInfo>(NaviFra::RobotInfo::KEY);
        NLOG(debug) << "RobotInfo retrieved" << std::endl;
        
        NLOG(debug) << "Getting RobotPose from InMemoryRepository..." << std::endl;
        auto robot_pose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
        NLOG(debug) << "RobotPose retrieved" << std::endl;

        NLOG(debug) << "Writing registers..." << std::endl;
        NLOG(debug) << "Writing COMMAND_NUM..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_COMMAND_NUM, NcCheonilPDUManager::instance().readRegister(PDUdefinition::PDU_READ_REGISTER_COMMAND_NUM));
        NLOG(debug) << "Writing COMMAND..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_COMMAND, NcCheonilPDUManager::instance().readRegister(PDUdefinition::PDU_READ_REGISTER_COMMAND));
        NLOG(debug) << "Writing X_POSITION..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_X_POSITION, static_cast<int16_t>(robot_pose->getPosition().x));
        NLOG(debug) << "Writing Y_POSITION..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_Y_POSITION, static_cast<int16_t>(robot_pose->getPosition().y));
        NLOG(debug) << "Writing ANGLE_POSITION..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_ANGLE_POSITION, static_cast<int16_t>(robot_pose->getPosition().deg));
        NLOG(debug) << "Writing CONFIDENCE..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_CONFIDENCE, static_cast<int16_t>(librobotInfo->getConfidence()));
        NLOG(debug) << "Writing DRIVE_STATUS..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_DRIVE_STATUS, static_cast<int16_t>(n_robot_status_));
        // NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_CURRENT_NODE, static_cast<int16_t>(librobotInfo->getNowNode()));
        // NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_TARGET_NODE, static_cast<int16_t>(librobotInfo->getGoalNode()));
        NLOG(debug) << "Writing CURRENT_SPEED..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_CURRENT_SPEED, static_cast<int16_t>(librobotInfo->getTwistLinearX()));
        NLOG(debug) << "Writing PATH_SPEED..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_PATH_SPEED, static_cast<int16_t>(librobotInfo->getRobotPathSpeed()));
        // NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_ALARM_CODE, static_cast<int16_t>(librobotInfo->getAlarmId()));
        NLOG(debug) << "Writing ANGLE_SPEED..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_ANGLE_SPEED, static_cast<int16_t>(librobotInfo->getTwistLinearZ()));
        // NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_NEXT_NODE, static_cast<int16_t>(librobotInfo->getNextNode()));
        // NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_TRACTION_MOTOR_CURRENT, librobotInfo->);
        // NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_STEER_MOTOR_CURRENT, librobotInfo->);
        // NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_STEER_DEGREE, librobotInfo->getSteerDegree());
        NLOG(debug) << "Writing AUTO_CHARGE..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_AUTO_CHARGE, static_cast<int16_t>(librobotInfo->getBatteryCmd()));
        // NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_TRACTION_MOTOR_RPM, librobotInfo->);
        // NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_STEER_MOTOR_RPM, librobotInfo->);
        NLOG(debug) << "Writing FORK_UP_DOWN_POSITION..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_FORK_UP_DOWN_POSITION, static_cast<int16_t>(librobotInfo->getForkUpDownPosition()));
        NLOG(debug) << "Writing FORK_UP_DOWN_CMD..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_FORK_UP_DOWN_CMD, static_cast<int16_t>(librobotInfo->getForkUpDownCmd()));
        NLOG(debug) << "Writing TILTING_UP_DOWN_CMD..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_TILTING_UP_DOWN_CMD, static_cast<int16_t>(librobotInfo->getTiltingUpDownCmd()));
        NLOG(debug) << "Writing FORK_WIDTH_CMD..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_FORK_WIDTH_CMD, static_cast<int16_t>(librobotInfo->getForkWidthCmd()));
        NLOG(debug) << "Writing LIGHT_CMD..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_LIGHT_CMD, static_cast<int16_t>(librobotInfo->getLight()));
        NLOG(debug) << "Writing LSC_FIELD..." << std::endl;
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_LSC_FIELD, static_cast<int16_t>(librobotInfo->getOSSDField()));
        int16_t n_has_task = 0;
        if(librobotInfo->getNowTask() != "idle") {
            n_has_task = 1;
        }
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_HAS_JOB, static_cast<int16_t>(n_has_task));

        int16_t n_has_pallet_id = 0;
        if(librobotInfo->getPalletID() == "NoRead") {
            n_has_pallet_id = 2;
        }
        else if(librobotInfo->getPalletID() != "") {
            n_has_pallet_id = 1;
        }
        NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_WRITE_REGISTER_HAS_ID, static_cast<int16_t>(n_has_pallet_id));
        NLOG(debug) << "All registers written" << std::endl;
        NLOG(debug) << "handleStatusPLC() - end" << std::endl;
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "handleStatusPLC response failed: " << e.displayText();
    }
    catch (const std::exception& e) {
        NLOG(error) << "handleStatusPLC std::exception: " << e.what();
    }
    catch (...) {
        NLOG(error) << "handleStatusPLC unknown exception";
    }
}

void NcCheonilStatusHandler::Status() // 로봇 상태 피엘씨에게 젇달ㄹ
{
    try {
        NLOG(debug) << "Status() - start" << std::endl;
        auto librobotInfo = InMemoryRepository::instance().get<NaviFra::RobotInfo>(NaviFra::RobotInfo::KEY);
        static RobotStatus previousRobotStatus = RobotStatus::IDLE;

        NLOG(debug) << "Status() - getting robot status string..." << std::endl;
        RobotStatus CurrentRobotStatus = stringToRobotStatus(librobotInfo->getStatus());
        NLOG(debug) << "Status() - robot status: " << static_cast<int>(CurrentRobotStatus) << std::endl;

        if (CurrentRobotStatus == RobotStatus::IDLE) {
            n_robot_status_ = 0;
        }
        else if (CurrentRobotStatus == RobotStatus::RUNNING) {
            n_robot_status_ = 1;
        }
        else if (CurrentRobotStatus == RobotStatus::PAUSED) {
            n_robot_status_ = 3;
        }
        else if (
            CurrentRobotStatus == RobotStatus::PausedByUser || CurrentRobotStatus == RobotStatus::PausedByObs ||
            CurrentRobotStatus == RobotStatus::PausedByPath) {
            n_robot_status_ = 3;
        }

        previousRobotStatus = CurrentRobotStatus;
        NLOG(debug) << "Status() - end" << std::endl;
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "Status failed: " << e.displayText();
    }
    catch (const std::exception& e) {
        NLOG(error) << "Status std::exception: " << e.what();
    }
    catch (...) {
        NLOG(error) << "Status unknown exception";
    }
}

void NcCheonilStatusHandler::ObstacleCheck() // 장애물 감지 첵
{
    auto librobotInfo = InMemoryRepository::instance().get<NaviFra::RobotInfo>(NaviFra::RobotInfo::KEY);
    auto& pdu = NcCheonilPDUManager::instance();

    auto auto_on = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_AUTO_ON);
    auto Lsc1 = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_LSC_1);
    auto Lsc2 = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_LSC_2);
    auto Lsc3 = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_LSC_3);
    auto Lsc4 = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_LSC_4);

    static bool Lsc_flag = false;
    static float prev_time = 0.0f;
    
    if((Lsc1 == 3 || Lsc2 == 3 || Lsc3 == 3 || Lsc4 == 3) && !Lsc_flag && auto_on == 1) 
    {      
        NaviFra::navifraCommand("pause");
        NLOG(info) << "=====================================";
        NLOG(info) << "             Robot Pause             ";
        NLOG(info) << "=====================================";

        Lsc_flag = true;
    } 
    else if ((Lsc1 != 3 && Lsc2 != 3 && Lsc3 != 3 && Lsc4 != 3) && Lsc_flag && auto_on == 1) 
    {
        NaviFra::navifraCommand("resume");
        // NLOG(info) << "=====================================";
        // NLOG(info) << "             Robot Resume            ";
        // NLOG(info) << "=====================================";

        Lsc_flag = false;
    }
    else if (((Lsc1 == 1 || Lsc1 == 2) || (Lsc2 == 1 || Lsc2 == 2) || (Lsc3 == 1 || Lsc3 == 2) || (Lsc4 == 1 || Lsc4 == 2)) && !Lsc_flag && auto_on == 1) 
    {
        NaviFra::NavifraSpeedLimit(0.2);
        NLOG(info) << "=====================================";
        NLOG(info) << "        Robot speed: 0.2 m/s         ";
        NLOG(info) << "=====================================";
    }
    else if((Lsc1 == 0 && Lsc2 == 0 && Lsc3 == 0 && Lsc4 == 0) && !Lsc_flag && auto_on == 1) 
    {      
        NaviFra::NavifraSpeedLimit(2.5);
        // NLOG(info) << "=====================================";
        // NLOG(info) << "        Robot speed: 2.5 m/s         ";
        // NLOG(info) << "=====================================";
    } 
}

void NcCheonilStatusHandler::JogMove() // 조그 앞 뒹
{
    try {
        NLOG(debug) << "JogMove() - start" << std::endl;
        auto& pdu = NcCheonilPDUManager::instance();

        NLOG(debug) << "JogMove() - reading JOG_ENABLE..." << std::endl;
        bool b_jog_enable = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOG_ENABLE); // 1: front 2: rear
        NLOG(debug) << "JogMove() - reading JOG_SPEED..." << std::endl;
        int n_linear_speed = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOG_SPEED); // 1: front 2: rear
        NLOG(debug) << "JogMove() - reading JOG_ANGLE_SPEED..." << std::endl;
        int n_angular_speed = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOG_ANGLE_SPEED); // 1: front 2: rear
        NLOG(debug) << "JogMove() - reading AUTO_ON..." << std::endl;
        bool b_auto_mode = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_AUTO_ON); // 1: front 2: rear
        
        if(b_jog_enable && !b_auto_mode)
        {
            NLOG(debug) << "JogMove() - calling cmdVel..." << std::endl;
            float f_linear_speed = n_linear_speed / 1000.0;
            float f_angular_speed = n_angular_speed * M_PI / 180.0 ;
            cmdVel(f_linear_speed, 0.0, f_angular_speed);
            NLOG(debug) << "JogMove() - cmdVel completed" << std::endl;
        }
        NLOG(debug) << "JogMove() - end" << std::endl;
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "JogMove failed: " << e.displayText();
    }
    catch (const std::exception& e) {
        NLOG(error) << "JogMove std::exception: " << e.what();
    }
    catch (...) {
        NLOG(error) << "JogMove unknown exception";
    }
}

// void NcCheonilStatusHandler::JogLeftRight() // 조그 오른쪾 왼쪽
// {
//     auto& pdu = NcCheonilPDUManager::instance();

//     static JogStatus pre_jog_left_right = PRE_JOG;

//     auto jog_front_rear = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOG_FRONT_REAR); // 1: front 2: rear
//     auto jog_left_right = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_JOG_LEFT_RIGHT); // 1. left 2:right

//     JogStatus jog_front_rear_status = static_cast<JogStatus>(jog_front_rear);
//     JogStatus jog_left_right_status = static_cast<JogStatus>(jog_left_right);

//     if(jog_front_rear_status == JOG_NONE && jog_left_right_status == JOG_NONE && jog_front_rear_status != pre_jog_left_right )
//     {
//         cmdVel(0.0, 0.0, 0.0);
//     }
//     else if(jog_left_right == JOG_FRONT_LEFT)
//     {
//         NaviFra::cmdVel(0.0, 0.0, 0.2);
//         NLOG(info) << "jog_left turn";
//     }
//     else if(jog_left_right == JOG_REAR_RIGHT)
//     {
//         NaviFra::cmdVel(0.0, 0.0, -0.2);
//         NLOG(info) << "jog_left turn";
//     }

//     pre_jog_left_right = jog_left_right_status;

// }

void NcCheonilStatusHandler::Charger() // 충전기 상태 확인
{
    try {
        NLOG(debug) << "Charger() - start" << std::endl;
        auto& pdu = NcCheonilPDUManager::instance();

        NLOG(debug) << "Charger() - reading CHARGE_STATE..." << std::endl;
        auto charger = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_CHARGE_STATE);
        NLOG(debug) << "Charger() - CHARGE_STATE: " << charger << std::endl;

        static bool b_charger_flag = false;

        if (charger == ChargerStatus::CHARGING && !b_charger_flag) { // CHARGING: 1 (충전중), UNCHARGING: 0 (충전 아님)
            NLOG(info) << "============================================";
            NLOG(info) << "             Charger On (충전중)            ";
            NLOG(info) << "============================================";
            b_charger_flag = true;
        }
        else if (charger == ChargerStatus::UNCHARGING && b_charger_flag) {
            NLOG(info) << "==================================================";
            NLOG(info) << "             Charger Off (충전중 아님)            ";
            NLOG(info) << "==================================================";
            b_charger_flag = false;
        }
        NLOG(debug) << "Charger() - end" << std::endl;
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "Charger failed: " << e.displayText();
    }
    catch (const std::exception& e) {
        NLOG(error) << "Charger std::exception: " << e.what();
    }
    catch (...) {
        NLOG(error) << "Charger unknown exception";
    }
}

void NcCheonilStatusHandler::ManualCharger() // Manual 일떄 PLC 신호 체크 후 자동으로 신호 값 변경
{
    try {
        NLOG(debug) << "ManualCharger() - start" << std::endl;
        auto& pdu = NcCheonilPDUManager::instance();

        NLOG(debug) << "ManualCharger() - reading AUTO_ON..." << std::endl;
        auto n_auto = pdu.readRegister(PDUdefinition::PDU_READ_REGISTER_AUTO_ON);
        NLOG(debug) << "ManualCharger() - AUTO_ON: " << n_auto << std::endl;

        int16_t n_battery_reset_cmd = 0;

        static int16_t n_previous_auto = 99;


        if(n_auto == RobotMode::ON && (n_previous_auto != n_auto)) // ON: 1 (신호 들어옴), OFF: 0 (신호 없음)
        {
            // CheonilBatteryCmd(n_battery_reset_cmd);

            NLOG(info) << "====================================================";
            NLOG(info) << "     Manual Charge Reset 충전기 상태 0으로 초기화 함 ";
            NLOG(info) << "====================================================";

        }

        n_previous_auto = n_auto;
        NLOG(debug) << "ManualCharger() - end" << std::endl;
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "ManualCharger failed: " << e.displayText();
    }
    catch (const std::exception& e) {
        NLOG(error) << "ManualCharger std::exception: " << e.what();
    }
    catch (...) {
        NLOG(error) << "ManualCharger unknown exception";
    }
}

// void NcCheonilStatusHandler::InPolygon()
// {
//     static bool prevInPolygon = false;
    
//     auto librobotInfo = InMemoryRepository::instance().get<NaviFra::RobotInfo>(NaviFra::RobotInfo::KEY);
//     bool currentInPolygon = librobotInfo->getInPolygon();

//     if (currentInPolygon != prevInPolygon)
//     {
//         NLOG(info) << "[CHANGED] InPolygon changed!"
//                 << " Prev(before update): " << prevInPolygon
//                 << " -> Current: " << currentInPolygon;

//         NLOG(info) << "[PDU WRITE] FRONT=" << currentInPolygon
//                 << ", LEFT=" << currentInPolygon
//                 << ", RIGHT=" << currentInPolygon;

//         NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_WRITE_COIL_FRONT_SAFETY_SCANNER, currentInPolygon);
//         NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_WRITE_COIL_LEFT_SAFETY_SCANNER, currentInPolygon);
//         NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_WRITE_COIL_RIGHT_SAFETY_SCANNER, currentInPolygon);

//         prevInPolygon = currentInPolygon;

//         NLOG(info) << "[UPDATED] Prev(after update): " << prevInPolygon;
//     }
// }

