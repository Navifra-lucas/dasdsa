// #ifndef NAVIFRA_CHEONIL_EMUL_STATE_H
// #define NAVIFRA_CHEONIL_EMUL_STATE_H

// #include "nc_cheonil_agent/state/state_machine.h"
// #include "nc_cheonil_agent/util/Timer.h"
// #include "nc_cheonil_agent/data/PDUdefinition.h"

// namespace NaviFra {
// enum class CheonilEmulState
// {
//     // IDLE,
//     // AMR_VALID,
//     // LOAD_REQ,
//     // UNLOAD_REQ,
//     // MOVE_REQ,
//     // CS_0,
//     // READY,
//     // BUSY,
//     // AUTO,
//     // MANUAL,
//     // ALARM,
//     // HOME,
//     // TR_REQ,
//     // COMPT,
//     // ALARM_BEFORE

//     IDLE,
//     TR_REQUEST,
//     BUSY,
//     COMPT
// };

// enum class CheonilControllerEmulTimeout
// {
//     CHEONIL_T1 = 2000,
//     CHEONIL_T2 = 2000,
//     CHEONIL_T3 = 2000,
//     CHEONIL_DEFAULT = 60000,
// };

// class CheonilEmulController : public StateMachine<CheonilEmulState> {
// public:
//     CheonilEmulController(CheonilEmulState initialState);
//     virtual ~CheonilEmulController();

//     void excute();

// protected:
//     virtual std::string implStateToString(CheonilEmulState state) const;

// private:
//     Util::Timer timer_;
// };
// }  // namespace Navifra

// #endif
