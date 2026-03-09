#include "ros1/graph_map/graph_map_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

bool GraphMapRos1::ResponseLoadMap(
    answer_msgs::CommonService::Request  &req,
    answer_msgs::CommonService::Response &res)
{
    LOG_INFO("[ResponseLoadMap] file: {}", req.data.c_str());

    o_graph_map_ptr_->SetFile(req.data);

    if (o_graph_map_ptr_->LoadMap())
    {
        res.success = true;
        res.reason = "";
    }
    else
    {
        res.success = false;
        res.reason = "fail to load map";
        LOG_ERROR("[ResponseLoadMap] Fail to load map");
    }

    return res.success;

}

} // namespace NVFR
