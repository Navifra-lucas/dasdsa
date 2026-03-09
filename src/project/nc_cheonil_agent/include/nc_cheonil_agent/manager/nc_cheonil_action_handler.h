#ifndef NC_CHEONIL_ACTION_HANDLER_H
#define NC_CHEONIL_ACTION_HANDLER_H

#include "nc_cheonil_agent/nc_cheonil_agent_pch.h"
#include "nc_cheonil_agent/manager/nc_cheonil_pdu_manager.h"
#include "nc_cheonil_agent/data/PDU.h"
#include "nc_cheonil_agent/data/PDUdefinition.h"
#include <core_agent/data/types.h>

namespace NaviFra {
class NcCheonilActionHandler {
public:
    NcCheonilActionHandler();
    ~NcCheonilActionHandler();

    void ReadCoilData();
    void ReadRegisterData();
    void handleActionPLC();
    void LeftLoad();
    void LeftUnLoad();
    void RightLoad();
    void RightUnLoad();
    void ChargingStart();
    void ChargingEnd();
    void Reset();
    void AlarmReset();
    void ConveyorCW();
    void ConveyorCCW();
    void ConveyorStop();
    void LeftStopperUp();
    void LeftStopperDown();
    void RightStopperUp();
    void RightStopperDown();
};
}

#endif // NC_CHEONIL_ACTION_HANDLER_H