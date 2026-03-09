

// #include "nc_cheonil_agent/emul/nc_cheonil_Emul_State.h"
// #include "nc_cheonil_agent/data/PDUdefinition.h"
// #include "nc_cheonil_agent/emul/nc_cheonil_emulpdu_manager.h"


// using namespace NaviFra;

// CheonilEmulController::CheonilEmulController(CheonilEmulState initialState)
//     : StateMachine::StateMachine(initialState)
//     , timer_(static_cast<long>(CheonilControllerEmulTimeout::CHEONIL_DEFAULT))
// {
//     try {
//         addTransition(
//             CheonilEmulState::IDLE, CheonilEmulState::TR_REQUEST,
//             [&]() {
//                 NLOG(info) << "Transitioning to EMUL - TR_REQUEST";
//                 NcCheonilEmulPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_TR_REQUEST, true);
//                 timer_.start(3000);
//             },
//             [&]() { return true; });

//         addTransition(
//             CheonilEmulState::TR_REQUEST, CheonilEmulState::BUSY,
//             [&]() {
//                 NcCheonilEmulPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_TR_REQUEST, false);
//                 NcCheonilEmulPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_BUSY, true);
//                 timer_.start(1000);
//             },
//             []() { return true; });

//         addTransition(
//             CheonilEmulState::BUSY, CheonilEmulState::COMPT,
//             [&]() {
//                 NcCheonilEmulPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_BUSY, false);
//                 NcCheonilEmulPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_COMPT, true);
//                 timer_.start(1000);
//             },
//             []() { return true; });

//         addTransition(
//             CheonilEmulState::COMPT, CheonilEmulState::IDLE,
//             [&]() {
//                 NcCheonilEmulPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_COMPT, false);
//             },
//             []() { return true; });

//         NcCheonilEmulPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_HO_AVBL, true);
//         NcCheonilEmulPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_EMERGENCY_STOP, false);
//     }
//     catch (Poco::Exception& ex) {
//         NLOG(error) << ex.displayText();
//     }
// }

// CheonilEmulController::~CheonilEmulController()
// {
// }

// void CheonilEmulController::excute()
// {
//     CheonilEmulState state = getState();

//     switch (state) {
//         case CheonilEmulState::IDLE:
//             if (NcCheonilEmulPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_CS)) {
//                 setState(CheonilEmulState::TR_REQUEST);
//             }
//             break;
//         case CheonilEmulState::TR_REQUEST:
//             if (NcCheonilEmulPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_UNLOAD_REQUEST)) {
//                 setState(CheonilEmulState::BUSY);
//             }
//             else if (NcCheonilEmulPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_LOAD_REQUEST)) {
//                 setState(CheonilEmulState::BUSY);
//             }
//             break;
//         case CheonilEmulState::BUSY:
//             if (timer_.isTimeout()) {
//                 setState(CheonilEmulState::COMPT);
//             }
//             break;
//         case CheonilEmulState::COMPT:
//             if (timer_.isTimeout()) {
//                 setState(CheonilEmulState::IDLE);
//             }
//             break;
//         default:
//             break;
//     }
// }

// std::string CheonilEmulController::implStateToString(CheonilEmulState state) const
// {
//     switch (state) {
//         case CheonilEmulState::IDLE:
//             return "IDLE";
//         case CheonilEmulState::TR_REQUEST:
//             return "TR_REQUEST";
//         case CheonilEmulState::BUSY:
//             return "BUSY";
//         case CheonilEmulState::COMPT:
//             return "COMPT";
//         default:
//             return "UNKNOWN";
//     }
// }