
/*
 * @file	: debug_visualizer.hpp
 * @date	: Feb. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: Navigation 알고리즘 Debug를 위해 만든 visualizer 클래스
 * @remark	: 싱글톤으로 만듬
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_DEGUB_VISUALIZER_H_
#define NAVIFRA_DEGUB_VISUALIZER_H_

#include "pos/pos.hpp"
#include "util/cbfunc_register.hpp"
#include "util/singleton_generator.hpp"

#include <iostream>
#include <vector>
using namespace std;

namespace NaviFra {
using namespace NaviFra;
struct ColorInfo {
    float f_a;
    float f_r;
    float f_g;
    float f_b;
    ColorInfo()
    {
        f_r = 0.f;
        f_g = 0.f;
        f_b = 0.f;
        f_a = 1.f;
    }
};
struct ScaleInfo {
    float f_scale_x_m;
    float f_scale_y_m;
    ScaleInfo()
    {
        f_scale_x_m = 0.1;
        f_scale_y_m = 0.1;
    }
};

struct DrawInfo {
    ColorInfo st_color;
    ScaleInfo st_scale;
};

struct VisMapInfo {
    std::string str_frame_id;
    float f_origin_x;
    float f_origin_y;
    float f_origin_orientation_w;
    int n_map_height;
    int n_map_width;
    float f_map_res;
    VisMapInfo()
    {
        str_frame_id = "map";
        f_origin_x = 0.0;
        f_origin_y = 0.0;
        f_origin_orientation_w = 1.0;
        n_map_height = 0;
        n_map_width = 0;
        f_map_res = 0.0;
    }
};

class DebugVisualizer : public SingletonGenerator<DebugVisualizer> {
public:
    enum VISUALIZE_TYPE
    {
        POS_VECTOR_LINE = 0,
        POS_CYLINDER,
        POS_MULTI_CYLINDER,
        POS_MULTI_CYLINDER1,
        POS_SPHERE,
        POS_ARROW,
        POS_PATH,
        FLOAT,
        POS_VECTOR_LINE_LOCAL,
        POLYGON,
        POS_VECTOR_ARROW,
        MAP,
    };

    DebugVisualizer();
    virtual ~DebugVisualizer();

    ScaleInfo GetScaleInfo(float f_x_m = 0.5, float f_y_m = 0.1);
    ColorInfo GetColorInfo(float f_r = 1.0, float f_g = 0.0, float f_b = 0.0, float f_a = 1.0);
    DrawInfo GetDrawInfo(ColorInfo st_color, ScaleInfo st_scale);

    /**
     * @brief 경로를 화면에 보여줄때 사용
     *
     * @param str_topic_name
     * @param vec_path
     * @param st_info
     */
    void PublishPath(const std::string& str_topic_name, const std::vector<Pos>& vec_path, const DrawInfo& st_info);
    void PublishLocalPath(const std::string& str_topic_name, const std::vector<Pos>& vec_path, const DrawInfo& st_info);

    /**
     * @brief nav_msgs path
     *
     * @param str_topic_name
     * @param vec_path
     */
    void PublishNavPath(const std::string& str_topic_name, const std::vector<Pos>& vec_path);

    /**
     * @brief float pub
     *
     * @param str_topic_name
     * @param str
     */
    void PublishFloat(const std::string& str_topic_name, const float& f_data);

    /**
     * @brief 실린더 형태로 특정 존 정보를 표시할때 사용
     *
     * @param str_topic_name
     * @param Pos
     * @param st_info
     */
    void PublishSingleZone(const std::string& str_topic_name, const Pos& Pos, const DrawInfo& st_info);

    /**
     * @brief 실린더 형태로 멀티 존 정보를 표시할때 사용
     *
     * @param str_topic_name
     * @param vec_pos
     * @param st_info
     */
    void PublishMultiZone(const std::string& str_topic_name, const vector<Pos>& vec_pos, const DrawInfo& st_info);

    /**
     * @brief 실린더 형태로 멀티 존 정보를 표시할때 사용
     *
     * @param str_topic_name
     * @param vec_pos
     * @param st_info
     */
    void PublishMultiZone1(const std::string& str_topic_name, const vector<Pos>& vec_pos, const DrawInfo& st_info);

    /**
     * @brief for rear or diagonal move
     *
     * @param str_topic_name
     * @param o_yaw_bias_pos
     * @param st_info
     */
    void PublishYawBias(const std::string& str_topic_name, const Pos& o_yaw_bias_pos, const DrawInfo& st_info);

    /**
     * @brief polygon 정보 표시할때 사용
     *
     * @param str_topic_name
     * @param vec_pos
     * @param st_info
     */
    void PublishPolygon(const std::string& str_topic_name, const vector<Pos>& vec_pos);
    void PublishPolygon(const std::string& str_topic_name, const vector<Pos>& vec_pos, const DrawInfo& st_info);
    void PublishArrowVector(const std::string& str_topic_name, const vector<Pos>& o_pos, const DrawInfo& st_info);
    void SetOffset(const Pos& o_offset_pos);

    Pos GetOffset();

    CBFuncRegister* GetCbRegister() { return &cb_register_; }

    void PublishRosMap(const std::string& str_topic_name, const vector<int8_t> vec_map, const VisMapInfo& st_info);

private:
    bool b_publish_flag_;
    CBFuncRegister cb_register_;
    Pos o_offset_pos_;
};
}  // namespace NaviFra

#endif
