// #ifndef NAVIFRA_CHEONIL_STATE_H
// #define NAVIFRA_CHEONIL_STATE_H

// #include "nc_cheonil_agent/state/state_machine.h"
// #include "nc_cheonil_agent/util/Timer.h"
// #include "nc_cheonil_agent/data/PDUdefinition.h"

// namespace NaviFra {
// enum class CheonilState
// {
//     IDLE,
//     AMR_VALID,
//     LOAD_REQ,
//     UNLOAD_REQ,
//     MOVE_REQ,
//     CS_0,
//     READY,
//     BUSY,
//     AUTO,
//     MANUAL,
//     ALARM,
//     HOME,
//     TR_REQ,
//     COMPT,
//     ALARM_BEFORE
// };

// enum class CheonilControllerTimeout
// {
//     CHEONIL_T1 = 30000,
//     CHEONIL_T2 = 2000,
//     CHEONIL_T3 = 2000,
//     CHEONIL_DEFAULT = 180000,
// };

// class CheonilController : public StateMachine<CheonilState> {
// public:
//     CheonilController(CheonilState initialState);
//     virtual ~CheonilController();

//     void excute();
// public:
//     bool erroralarm_ = false;
//     bool timeoutalarm_ = false;

//     bool loadcheck_ = false;
//     bool unloadcheck_ = false;

//     int target_level_ = 0;
//     int current_level_ = 0;

//     bool cheonil_action_ = false;
//     bool b_cheonil_status_ = false;
//     std::string cheonil_mode_ = "";

//     void setCheonilAction(bool action);
//     void setCheonilLevel(int level);
//     void setCheonilMode(std::string value);

// private:
//     void addSequenceCommon();
//     void addSequenceUnload();
//     void addSequenceload();

//     bool avlemergencycheck();
//     void idlestatus();
//     void opendoorstatus();
//     void CommonTransitions();
//     void configureTransitions();
//     void resetAllCoils();
//     void addCommonTransitions(CheonilState initialState);

// protected:
//     virtual std::string implStateToString(CheonilState state) const;

// private:
//     Util::Timer timer_;
//     bool callrequest_ = false;
//     bool loadrequest_ = false;
//     bool unloadrequest_ = false;
// };
// }  // namespace Navifra

// #endif
