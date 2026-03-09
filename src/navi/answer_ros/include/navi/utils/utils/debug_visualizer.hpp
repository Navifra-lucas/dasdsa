
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

#include "utils/cbfunc_register.hpp"
#include "utils/singleton_generator.hpp"
#include "utils/pose.hpp"
#include "utils/grid_map/map_info.hpp"

#include <iostream>
#include <vector>
using namespace std;

namespace NVFR {

struct ColorInfo {
    float f_a;
    float f_r;
    float f_g;
    float f_b;
    ColorInfo()
    {
        f_r = 0.f;
        f_g = 1.f;
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

class DebugVisualizer : public NaviFra::SingletonGenerator<DebugVisualizer> {
public:
    /**
     * @note type: int
     * @param POSE, LOCAL_POSE, MULTI_POSE, LOCAL_MULTI_POSE, PATH, LOCAL_PATH, POLYGON, LOCAL_POLYGON, MULTI_POLYGON, LOCAL_MULTI_POLYGON, MAP
     */
    enum VISUALIZE_TYPE : int
    {
        POSE = 0,
        LOCAL_POSE = 1,
        MULTI_POSE,
        LOCAL_MULTI_POSE,
        PATH,
        LOCAL_PATH,
        POLYGON,
        LOCAL_POLYGON,
        MULTI_POLYGON,
        LOCAL_MULTI_POLYGON,
        GLOBAL_MAP,
        LOCAL_MAP,
    };

    DebugVisualizer();
    virtual ~DebugVisualizer();

    ScaleInfo GetScaleInfo(float f_x_m = 0.5, float f_y_m = 0.1);
    ColorInfo GetColorInfo(float f_r = 1.0, float f_g = 0.0, float f_b = 0.0, float f_a = 1.0);
    DrawInfo GetDrawInfo(ColorInfo st_color, ScaleInfo st_scale);

    /**
     * @brief 실린더 형태로 특정 존 정보를 표시할때 사용
     *
     * @param s_topic_name
     * @param Pose
     * @param st_info
     */
    void PublishPose(const string& s_topic_name, const Pose& pose, const DrawInfo& st_info);
    void PublishLocalPose(const string& s_topic_name, const Pose& pose, const DrawInfo& st_info);

    /**
     * @brief 실린더 형태로 멀티 존 정보를 표시할때 사용
     *
     * @param s_topic_name
     * @param vec_pos
     * @param st_info
     */
    void PublishMultiPose(const string& s_topic_name, const vector<Pose>& vec_pose, const DrawInfo& st_info);
    void PublishLocalMultiPose(const string& s_topic_name, const vector<Pose>& vec_pose, const DrawInfo& st_info);

    /**
     * @brief 경로를 화면에 보여줄때 사용
     *
     * @param s_topic_name
     * @param vec_path
     * @param st_info
     */
    void PublishPath(const string& s_topic_name, const Path& path, const DrawInfo& st_info);
    void PublishNavPath(const string& s_topic_name, const Path& path);
    void PublishLocalPath(const string& s_topic_name, const Path& path, const DrawInfo& st_info);
    void PublishLocalNavPath(const string& s_topic_name, const Path& path);

    /**
     * @brief polygon 정보 표시할때 사용
     *
     * @param s_topic_name
     * @param polygon
     * @param st_info
     */
    void PublishPolygon(const string& s_topic_name, const vector<Pose>& polygon);
    void PublishPolygon(const string& s_topic_name, const vector<Pose>& polygon, const DrawInfo& st_info);
    void PublishLocalPolygon(const string& s_topic_name, const vector<Pose>& polygon);
    void PublishLocalPolygon(const string& s_topic_name, const vector<Pose>& polygon, const DrawInfo& st_info);

    /**
     * @brief polygon 정보 표시할때 사용
     *
     * @param s_topic_name
     * @param vec_polygon
     * @param st_info
     */
    void PublishMultiPolygon(const string& s_topic_name, const vector< vector<Pose> >& vec_polygon);
    void PublishMultiPolygon(const string& s_topic_name, const vector< vector<Pose> >& vec_polygon, const DrawInfo& st_info);
    void PublishLocalMultiPolygon(const string& s_topic_name, const vector< vector<Pose> >& vec_polygon);
    void PublishLocalMultiPolygon(const string& s_topic_name, const vector< vector<Pose> >& vec_polygon, const DrawInfo& st_info);

    NaviFra::CBFuncRegister* GetCbRegister() { return &cb_register_; }

    void PublishGlobalMap(const string& s_topic_name, const vector<int8_t> map, const MapInfo_t& st_info);
    void PublishLocalMap(const string& s_topic_name, const vector<int8_t> map, const MapInfo_t& st_info);

private:
    NaviFra::CBFuncRegister cb_register_;
};

}  // namespace NVFR

#endif
