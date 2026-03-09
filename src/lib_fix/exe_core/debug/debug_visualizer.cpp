#include "debug_visualizer.hpp"

#include "util/logger.hpp"

namespace NaviFra {
DebugVisualizer::DebugVisualizer()
    : b_publish_flag_(true)
{
    LOG_INFO("Constructor");
}

DebugVisualizer::~DebugVisualizer()
{
    LOG_INFO("Desconstructor");
}

ScaleInfo DebugVisualizer::GetScaleInfo(float f_x_m, float f_y_m)
{
    ScaleInfo st_info;
    st_info.f_scale_x_m = f_x_m;
    st_info.f_scale_y_m = f_y_m;
    return st_info;
}

ColorInfo DebugVisualizer::GetColorInfo(float f_r, float f_g, float f_b, float f_a)
{
    ColorInfo st_info;
    st_info.f_r = f_r;
    st_info.f_g = f_g;
    st_info.f_b = f_b;
    st_info.f_a = f_a;
    return st_info;
}

void DebugVisualizer::SetOffset(const Pos& o_offset_pos)
{
    o_offset_pos_ = o_offset_pos;
}

Pos DebugVisualizer::GetOffset()
{
    return o_offset_pos_;
}

DrawInfo DebugVisualizer::GetDrawInfo(ColorInfo st_color, ScaleInfo st_scale)
{
    DrawInfo st_info;
    st_info.st_color = st_color;
    st_info.st_scale = st_scale;
    return st_info;
}

void DebugVisualizer::PublishPath(const std::string& str_topic_name, const std::vector<Pos>& vec_path, const DrawInfo& st_info)
{
    if (false == b_publish_flag_)
        return;
    bool b_ret_val = cb_register_.notify(POS_VECTOR_LINE, str_topic_name, vec_path, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishLocalPath(const std::string& str_topic_name, const std::vector<Pos>& vec_path, const DrawInfo& st_info)
{
    if (false == b_publish_flag_)
        return;
    bool b_ret_val = cb_register_.notify(POS_VECTOR_LINE_LOCAL, str_topic_name, vec_path, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishNavPath(const std::string& str_topic_name, const std::vector<Pos>& vec_path)
{
    if (false == b_publish_flag_)
        return;
    DrawInfo st_info;
    bool b_ret_val = cb_register_.notify(POS_PATH, str_topic_name, vec_path, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishFloat(const std::string& str_topic_name, const float& f_data)
{
    if (false == b_publish_flag_)
        return;
    DrawInfo st_info;
    bool b_ret_val = cb_register_.notify(FLOAT, str_topic_name, f_data, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishMultiZone(const std::string& str_topic_name, const vector<Pos>& vec_pos, const DrawInfo& st_info)
{
    if (false == b_publish_flag_)
        return;
    bool b_ret_val = cb_register_.notify(POS_MULTI_CYLINDER, str_topic_name, vec_pos, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishMultiZone1(const std::string& str_topic_name, const vector<Pos>& vec_pos, const DrawInfo& st_info)
{
    if (false == b_publish_flag_)
        return;
    bool b_ret_val = cb_register_.notify(POS_MULTI_CYLINDER1, str_topic_name, vec_pos, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishSingleZone(const std::string& str_topic_name, const Pos& Pos, const DrawInfo& st_info)
{
    if (false == b_publish_flag_)
        return;
    bool b_ret_val = cb_register_.notify(POS_CYLINDER, str_topic_name, Pos, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishYawBias(const std::string& str_topic_name, const Pos& o_yaw_bias_pos, const DrawInfo& st_info)
{
    if (false == b_publish_flag_)
        return;
    bool b_ret_val = cb_register_.notify(POS_ARROW, str_topic_name, o_yaw_bias_pos, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishPolygon(const std::string& str_topic_name, const vector<Pos>& vec_pos)
{
    if (false == b_publish_flag_)
        return;
    DrawInfo st_info;
    bool b_ret_val = cb_register_.notify(POLYGON, str_topic_name, vec_pos, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishPolygon(const std::string& str_topic_name, const vector<Pos>& vec_pos, const DrawInfo& st_info)
{
    if (false == b_publish_flag_)
        return;
    bool b_ret_val = cb_register_.notify(POLYGON, str_topic_name, vec_pos, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishArrowVector(const std::string& str_topic_name, const vector<Pos>& vec_pos, const DrawInfo& st_info)
{
    if (false == b_publish_flag_)
        return;
    bool b_ret_val = cb_register_.notify(POS_VECTOR_ARROW, str_topic_name, vec_pos, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}

void DebugVisualizer::PublishRosMap(const std::string& str_topic_name, const vector<int8_t> vec_map, const VisMapInfo& st_info)
{
    if (false == b_publish_flag_)
        return;
    bool b_ret_val = cb_register_.notify(MAP, str_topic_name, vec_map, st_info);
    if (false == b_ret_val) {
        LOG_INFO("Topic: %s Notify failure!!! ", str_topic_name.c_str());
    }
}
}  // namespace NaviFra
