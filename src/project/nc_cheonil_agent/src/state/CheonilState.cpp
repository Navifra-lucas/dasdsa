// #include "nc_cheonil_agent/state/CheonilState.h"
// #include "nc_cheonil_agent/state/state_machine.h"
// #include "nc_cheonil_agent/manager/nc_cheonil_pdu_manager.h"
// #include "nc_cheonil_agent/manager/nc_cheonil_manager.h"
// #include "nc_cheonil_agent/data/PDUdefinition.h"
// #include <core_agent/core/navicore.h>

// using namespace NaviFra;

// CheonilController::CheonilController(CheonilState initialState)
//     : StateMachine::StateMachine(initialState)
//     , timer_(static_cast<long>(CheonilControllerTimeout::CHEONIL_DEFAULT))
// {
//     try {
//         addSequenceCommon();
//         addSequenceUnload();
//         addSequenceload();

//         /// ALARM
//         configureTransitions();
//     }
//     catch (Poco::Exception& ex) {
//         NLOG(error) << ex.displayText();
//     }
// }

// CheonilController::~CheonilController()
// {
// }


// void CheonilController::addSequenceCommon()
// {
//     addTransition(
//         CheonilState::IDLE, CheonilState::AMR_VALID,
//         [&]() {
//             NLOG(info) << "Transitioning to AMR_VALID";
//             cheonil_action_ = false;
//             NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_AMR_VALID, true);
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_DEFAULT));
//             // timer_.start(3000);
//         },
//         [&]() {
//             if(avlemergencycheck())
//             {
//                 return true;
//             }
//             else
//             {
//                 return false;
//             }
//         });

//     addTransition(
//         CheonilState::AMR_VALID, CheonilState::CS_0,
//         [&]() {
//             NLOG(info) << "Transitioning to CS_0";
//             NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_CS, true);
//             NcCheonilPDUManager::instance().writeRegister(PDUdefinition::PDU_REGISTER_TARGET_LEVEL, target_level_);
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_DEFAULT));
//             b_cheonil_status_ = true;
//             // cheonilLiftStatus(b_cheonil_status_);
//             // timer_.start(1000);
//         },
//         [&]() { 
//             NLOG(info) << "cheonil_mode_: " << cheonil_mode_;
//             if (cheonil_mode_ == "unload" || cheonil_mode_ == "load")
//             {
//                 return true;
//             }
//             else
//             {
//                 NLOG(error) << "You can choose cheonil mode load or unload";
//                 setState(CheonilState::ALARM_BEFORE);
//                 return false;
//             }

//             NLOG(info) << "target_level_: " << target_level_;
//             if (target_level_)
//             {
//                 return true;
//             }
//             else
//             {
//                 NLOG(error) << "Cheonil level cannot be set to 0 or a negative value";
//                 setState(CheonilState::ALARM_BEFORE);
//                 return false;
//             }
//             });

//     addTransition(
//         CheonilState::CS_0, CheonilState::TR_REQ,
//         [&]() {
//             NLOG(info) << "Transitioning to TR_REQ";
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_DEFAULT));
//             // timer_.start(1000);
//         },
//         []() { return NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_TR_REQUEST); });
    
//     addTransition(
//         CheonilState::BUSY, CheonilState::COMPT, [&]() { 
//             NLOG(info) << "Transitioning to COMPT";
//             NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_AMR_VALID, false);
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_DEFAULT));
//         }, 
//         []() { return NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_COMPT); });

//     addTransition(
//         CheonilState::COMPT, CheonilState::IDLE, [&]() { 
//             Poco::Thread::sleep(300);//wenedy_test
//             b_cheonil_status_ = false;
//             // cheonilLiftStatus(b_cheonil_status_);
//             NLOG(info) << "target_level : " << target_level_ << " / current_level : " << current_level_;
//             NLOG(info) << "Transitioning to IDLE";
//             resetAllCoils();
//         }, 
//         [this]() { return target_level_== current_level_; });
//         // [this]() { return true; }); 

// }

// void CheonilController::addSequenceUnload()
// {
//     addTransition(
//         CheonilState::TR_REQ, CheonilState::UNLOAD_REQ,
//         [&]() {
//             NLOG(info) << "Transitioning to UNLOAD_REQ";
//             NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_UNLOAD_REQUEST, true);
//             NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_CS, false);
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_DEFAULT));
//             // timer_.start(1000);
//         },
//         []() { return true; });

//     addTransition(
//         CheonilState::UNLOAD_REQ, CheonilState::BUSY,
//         [&]() {
//             NLOG(info) << "Transitioning to BUSY";
//             NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_UNLOAD_REQUEST, false);
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_DEFAULT));
//             // timer_.start(1000);
//         },
//         []() { return NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_BUSY);});
// }

// void CheonilController::addSequenceload()
// {    
//     addTransition(
//         CheonilState::TR_REQ, CheonilState::LOAD_REQ,
//         [&]() {
//             NLOG(info) << "Transitioning to LOAD_REQ";
//             NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_LOAD_REQUEST, true);
//             NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_CS, false);
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_DEFAULT));
//             // timer_.start(1000);
//         },
//         []() { return true; });

//     addTransition(
//         CheonilState::LOAD_REQ, CheonilState::BUSY,
//         [&]() {
//             NLOG(info) << "Transitioning to BUSY";
//             NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_LOAD_REQUEST, false);
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_DEFAULT));
//             // timer_.start(1000);
//         },
//         []() { return NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_BUSY);});
// }

// void CheonilController::CommonTransitions()
// {
// }

// void CheonilController::excute()
// {
//     CheonilState state = getState();
//     bool ho_avbl = NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_HO_AVBL);
//     bool emergency_stop = NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_EMERGENCY_STOP);
    
//     // bool call_request = NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_CALL_REQUEST);
//     bool load_request = NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_LOAD_REQUEST);
//     bool unload_request = NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_UNLOAD_REQUEST);

//     current_level_ = NcCheonilPDUManager::instance().readRegister(PDUdefinition::PDU_REGISTER_CURRENT_LEVEL);

//     if (timer_.isTimeout() && state != CheonilState::ALARM_BEFORE && state != CheonilState::IDLE && state != CheonilState::ALARM ) {
//         if(!timeoutalarm_)
//         {
//             NLOG(error) << "State Timeout currnetState [ " << implStateToString(state) << " ]";
//             setState(CheonilState::ALARM_BEFORE);
//         }
//         timeoutalarm_ = true;
//     }
//     else if (state != CheonilState::IDLE && (!ho_avbl || emergency_stop))
//     {
//         if(!erroralarm_)
//         {
//             // logger_.error(Poco::format(
//                 // "Entering ALARM_BEFORE state due to HO_AVBL [ %s ] , ES [ %s ]", std::string(ho_avbl ? "true" : "false"),
//                 // std::string(emergency_stop ? "true" : "false")));
//             NLOG(error) << "Entering ALARM_BEFORE state due to HO_AVBL [ " << std::string(ho_avbl ? "true" : "false") << 
//                 " ] , ES [ " << std::string(emergency_stop ? "true" : "false") <<" ]";

//             setState(CheonilState::ALARM_BEFORE);
//         }
//         erroralarm_ = true;
//     }
//     else
//     {
//         timeoutalarm_ = false;
//         erroralarm_ = false;
//     }

//     switch (state) {
//         case CheonilState::IDLE:
//             if (cheonil_action_)
//             {
//                 NLOG(info) << "cheonil_action is true";
//                 setState(CheonilState::AMR_VALID);
//             }
//             break;
//         case CheonilState::AMR_VALID:
//             setState(CheonilState::CS_0);
//             // if (timer_.isTimeout())
//             // {
//             //     setState(CheonilState::CS_0);
//             // }
//             break;
//         case CheonilState::CS_0:
//             setState(CheonilState::TR_REQ);
//             // if (timer_.isTimeout())
//             // {
//             //     setState(CheonilState::TR_REQ);
//             // }
//             break;
//         case CheonilState::TR_REQ:
//             idlestatus();
//             break;
//         //UnLoad/Load Request
//         case CheonilState::UNLOAD_REQ:
//             setState(CheonilState::BUSY);
//             // if (timer_.isTimeout())
//             // {
//             //     setState(CheonilState::BUSY);
//             // }
//             break;
//         case CheonilState::LOAD_REQ:
//             setState(CheonilState::BUSY);
//             // if (timer_.isTimeout())
//             // {
//             //     setState(CheonilState::BUSY);
//             // }
//             break;
//         case CheonilState::BUSY:
//             setState(CheonilState::COMPT);
//             break;
//         case CheonilState::COMPT:
//             setState(CheonilState::IDLE);
//             break;
//         case CheonilState::ALARM_BEFORE:
//             if (timer_.isTimeout()) {
//                 setState(CheonilState::ALARM);
//             }
//             break;
//         case CheonilState::ALARM:
//             setState(CheonilState::IDLE);
//             break;
//         default:
//             break;
//     }
// }

// std::string CheonilController::implStateToString(CheonilState state) const
// {
//     switch (state) {
//         case CheonilState::IDLE:
//             return "IDLE";
//         case CheonilState::AMR_VALID:
//             return "AMR_VALID";
//         case CheonilState::CS_0:
//             return "CS_0";
//         case CheonilState::TR_REQ:
//             return "TR_REQ";
//         case CheonilState::UNLOAD_REQ:
//             return "UNLOAD_REQ";
//         case CheonilState::LOAD_REQ:
//             return "LOAD_REQ";
//         case CheonilState::BUSY:
//             return "BUSY";
//         case CheonilState::COMPT:
//             return "COMPT";
//         case CheonilState::ALARM_BEFORE:
//             return "ALARM_BEFORE";
//         case CheonilState::ALARM:
        
//         default:
//             return "UNKNOWN";
//     }
// }

// //test 함수 모음
// bool CheonilController::avlemergencycheck()
// {
//     bool ho_avbl = NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_HO_AVBL);
//     bool emergency_stop = NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_EMERGENCY_STOP);
//     if (ho_avbl && !emergency_stop) {
//         return true;
//     }
//     else {
//         // logger_.error(Poco::format(
//             // "can't request call HO_AVBL [ %s ] , ES [ %s ]", std::string(ho_avbl ? "true" : "false"),
//             // std::string(emergency_stop ? "true" : "false")));
//             NLOG(error) << "Entering ALARM_BEFORE state due to HO_AVBL [ " << std::string(ho_avbl ? "true" : "false") << 
//                 " ] , ES [ " << std::string(emergency_stop ? "true" : "false") <<" ]";
//         return false;
//     }
// }
// void CheonilController::idlestatus()
// {
//     NLOG(info) << "cheonil_mode : " << cheonil_mode_;

//     if(cheonil_mode_ == "unload")
//     {   
//         setState(CheonilState::UNLOAD_REQ);
//     } 
//     else if(cheonil_mode_ == "load")
//     {
//         setState(CheonilState::LOAD_REQ);
//     }
// }

// void CheonilController::configureTransitions()
// {
//     addTransition(
//         {CheonilState::IDLE, CheonilState::AMR_VALID,  CheonilState::TR_REQ, CheonilState::LOAD_REQ, CheonilState::UNLOAD_REQ,
//          CheonilState::CS_0, CheonilState::BUSY,CheonilState::COMPT},
//         CheonilState::ALARM_BEFORE,
//         [&] { resetAllCoils(); 
//             NLOG(error) << "Cheonil Error";
//             NLOG(info) << "Transitioning to ALARM_BEFORE";
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_T1));
//             },
//         // []() { return NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_EMERGENCY_STOP); } // test
//         []() { return true; } // test
//     );
//     addTransition(
//         CheonilState::ALARM_BEFORE, CheonilState::ALARM,
//         [&]() {
//             NLOG(info) << "Transitioning to ALARM";
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_T1));
//         },
//         // []() { return !NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_EMERGENCY_STOP); }); //test
//     addTransition(
//         CheonilState::ALARM, CheonilState::IDLE,
//         [&]() {
//             NLOG(info) << "Transitioning to IDLE";
//             timer_.start(static_cast<long>(CheonilControllerTimeout::CHEONIL_T1));
//         },
//         // []() { return !NcCheonilPDUManager::instance().readCoil(PDUdefinition::PDU_COIL_EMERGENCY_STOP); }); //test
// }

// void CheonilController::resetAllCoils()
// {
//     NLOG(info) << "Resetting all coils";

//     // NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_AMR_VALID, false);
//     // NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_LOAD_REQUEST, false);
//     // NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_UNLOAD_REQUEST, false);
//     // NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_TR_REQUEST, false);
//     // NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_BUSY, false);
//     // NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_CS, false);
//     // NcCheonilPDUManager::instance().writeCoil(PDUdefinition::PDU_COIL_COMPT, false);
// }

// void CheonilController::setCheonilMode(std::string value)
// {
//     cheonil_mode_ = value;
// }
// void CheonilController::setCheonilLevel(int level)
// {
//     target_level_ = level;
// }
// void CheonilController::setCheonilAction(bool action)
// {
//     cheonil_action_ = action;
// }