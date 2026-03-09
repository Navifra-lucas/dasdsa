#include <nc_wia_agent/action/nc_action_hplc_offset.h>

using namespace NaviFra;

NcActionPlcData::NcActionPlcData()
{
}
NcActionPlcData::~NcActionPlcData()
{
}

std::string NcActionPlcData::implName()
{
    return "NcActionPlcData";
}

void NcActionPlcData::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        if (!obj || !obj->has("plcdatas"))
            return;

        Poco::JSON::Array::Ptr plcdatas = obj->getArray("plcdatas");
        if (!plcdatas)
            return;

        for (size_t i = 0; i < plcdatas->size(); ++i) {
            Poco::JSON::Object::Ptr plc = plcdatas->getObject(i);
            if (!plc || !plc->has("data"))
                continue;

            Poco::JSON::Array::Ptr data_arr = plc->getArray("data");
            if (!data_arr)
                continue;

            std::vector<float> vec_offset_val;
            float f_offset_x = 0.0f;
            float f_offset_y = 0.0f;
            float f_offset_theta = 0.0f;
            bool b_has_offset_x = false;
            bool b_has_offset_y = false;
            bool b_has_offset_theta = false;

            for (size_t j = 0; j < data_arr->size(); ++j) {
                Poco::JSON::Object::Ptr item = data_arr->getObject(j);
                if (!item || !item->has("tag") || !item->has("val"))
                    continue;

                std::string tag = item->getValue<std::string>("tag");

                if (tag.find("offset_x") == 0) {
                    std::string val_str = item->getValue<std::string>("val");
                    f_offset_x = std::stof(val_str);
                    b_has_offset_x = true;
                    NLOG(info) << "[PLC] " << tag << " / " << f_offset_x;
                    continue;
                }
                
                if (tag.find("offset_y") == 0) {
                    std::string val_str = item->getValue<std::string>("val");
                    f_offset_y = std::stof(val_str);
                    b_has_offset_y = true;
                    NLOG(info) << "[PLC] " << tag << " / " << f_offset_y;
                    continue;
                }
                
                if (tag.find("offset_theta") == 0) {
                    std::string val_str = item->getValue<std::string>("val");
                    f_offset_theta = std::stof(val_str);
                    b_has_offset_theta = true;
                    NLOG(info) << "[PLC] " << tag << " / " << f_offset_theta;
                    continue;
                }
            }

            if (b_has_offset_x && b_has_offset_y && b_has_offset_theta) {
                vec_offset_val.push_back(f_offset_x);
                vec_offset_val.push_back(f_offset_y);
                vec_offset_val.push_back(f_offset_theta);
                status->setWingbodyOffset(vec_offset_val);
                NLOG(info) << "[PLC] setWingbodyOffset called with: x=" << f_offset_x 
                           << ", y=" << f_offset_y << ", theta=" << f_offset_theta;
            }
        }
    }
    catch (const std::exception& e) {
        NLOG(error) << "[PLC] Exception: " << e.what();
    }
}

