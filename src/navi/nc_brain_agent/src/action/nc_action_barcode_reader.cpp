#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_info.h>
#include <nc_brain_agent/action/nc_action_barcode_reader.h>

using namespace NaviFra;

NcActionBarcodeReader::NcActionBarcodeReader()
{
}

NcActionBarcodeReader::~NcActionBarcodeReader()
{
}

std::string NcActionBarcodeReader::implName()
{
    return "NcActionBarcodeReader";
}

void NcActionBarcodeReader::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    Poco::JSON::Object::Ptr data = obj->getObject("data");

    if (data->has("on")) {
        bool barcodeOnOff = data->get("on").convert<bool>();

        /*
            피터 숙제
        */
        InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setBarcodeOn(barcodeOnOff);
        barcodeReader(barcodeOnOff);
    }
    sendResponseSuccess(source, obj->get("uuid").extract<std::string>());
}