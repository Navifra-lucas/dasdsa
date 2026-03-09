#include "map_based_path_planner.hpp"

#include "core/util/logger.hpp"

namespace NaviFra {
MapPlanner::MapPlanner()
{
}
MapPlanner::~MapPlanner()
{
}

void MapPlanner::SetMap(const std::vector<NaviNode>& vec_topological_map)
{
    LOG_INFO("vec_topological_map size : %d", vec_topological_map.size());
    vec_topology_node_ = vec_topological_map;
    return;
}

void MapPlanner::SetMap(const NaviFra::Map& o_navi_map)  // o_navi_map_ = local map
{
    // std::shared_ptr<NaviFra::Map> o_sensor_map = std::make_shared<NaviFra::SensorMap>(o_map.GetXm(), o_map.GetYm(), -o_map.GetXm()/2,
    // -o_map.GetYm()/2, o_map.getresolutionM());
    std::lock_guard<std::mutex> lock(mtx_set_map_);
    o_navi_map_ = std::make_shared<NaviFra::Map>(o_navi_map);
    o_astar_algorithm_.SetMap(o_navi_map_);
    o_path_smoothing_.SetMap(o_navi_map_);
    f_ori_x_ = o_navi_map_->GetXOriginM();
    f_ori_y_ = o_navi_map_->GetYOriginM();
    f_res_ = o_navi_map_->getresolutionM();
    return;
}

void MapPlanner::SetBasePath(std::vector<NaviFra::Pos>* vec_path_ptr)
{
    o_astar_algorithm_.SetBasePath(vec_path_ptr);
}

bool MapPlanner::ExistObstacleOnLocalGoal(const NaviFra::Pos& o_local_goal)
{
    if (o_navi_map_ == nullptr)
        return false;

    float f_map_resolution = f_res_;
    int n_x_pixel = o_navi_map_->GetXpx();
    int n_y_pixel = o_navi_map_->GetYpx();

    int n_grid_x = (o_local_goal.GetXm() / f_map_resolution) + n_x_pixel / 2;
    int n_grid_y = (o_local_goal.GetYm() / f_map_resolution) + n_y_pixel / 2;

    // if(n_grid_x < 0) n_grid_x = 0;
    // if(n_grid_x >= n_x_pixel) n_grid_x = n_x_pixel - 1;
    // if(n_grid_y< 0) n_grid_y = 0;
    // if(n_grid_y >= n_y_pixel) n_grid_y = n_y_pixel - 1;

    std::lock_guard<std::mutex> lock(mtx_set_map_);
    std::shared_ptr<std::vector<int8_t>> ptr_map_ = std::make_shared<std::vector<int8_t>>(o_navi_map_->GetMap());

    // if (n_grid_x < 0 || n_grid_x >= n_x_pixel || n_grid_y < 0 || n_grid_y >= n_y_pixel) {
    //     LOG_ERROR("[HENRY] Out Of Map (local target) : %d %d %d %d", n_grid_x, n_grid_y,  n_x_pixel,  n_y_pixel);
    //     return true;
    // }

    // if (ptr_map_->at(n_grid_x + n_grid_y * n_x_pixel) >= static_cast<char>(100)) {
    //     return true;
    // }

    float padding_size = 3.0;

    for (int padding_y = -padding_size; padding_y <= padding_size; padding_y++) {
        for (int padding_x = -padding_size; padding_x <= padding_size; padding_x++) {
            int pad_grid_x = n_grid_x + padding_x;
            int pad_grid_y = n_grid_y + padding_y;

            if (pad_grid_x < 0 || pad_grid_x >= n_x_pixel || pad_grid_y < 0 || pad_grid_y >= n_y_pixel) {
                // LOG_ERROR("@@@@ %d %d %d %d", n_grid_x, n_grid_y,  n_x_pixel,  n_y_pixel);
                return true;
            }

            // char c_tmp_val;
            int8_t n8_tmp_val;

            // c_tmp_val = ptr_map_->at(pad_grid_x + pad_grid_y * n_x_pixel);
            n8_tmp_val = ptr_map_->at(pad_grid_x + pad_grid_y * n_x_pixel);
            // if (c_tmp_val >= static_cast<char>(100)) {
            if (n8_tmp_val >= (100)) {
                return true;
            }
        }
    }

    return false;
}

// if(n_grid_x < 0 || n_grid_x >= n_x_pixel || n_grid_y < 0 || n_grid_y >= n_y_pixel )
// {
//     LOG_ERROR("@@@@ %d %d %d %d", n_grid_x, n_grid_y,  n_x_pixel,  n_y_pixel);

//     return true;
// }

// char c_tmp_val;
// c_tmp_val = ptr_map_->at(n_grid_x + n_grid_y * n_x_pixel);

// if(c_tmp_val >= static_cast<char>(100))
// {
//     return true;
// }
// return false;

//   void MapPlanner::SaveMap2(int size_x,int size_y, std::make_shared<std::vector<char>> prob_map, const string function_name, int x1, int
//   y1)
void MapPlanner::SaveMap2(int size_x, int size_y, const string function_name, int x1, int y1)
{
    std::shared_ptr<std::vector<int8_t>> ptr_map_ = std::make_shared<std::vector<int8_t>>(o_navi_map_->GetMap());
    auto save = cv::Mat(size_y, size_x, CV_8UC1, cv::Scalar(205));
    int i = 0;
    for (int y = 0; y < size_y; y++) {
        for (int x = 0; x < size_x; x++) {
            int map_index = x + y * size_x;
            auto data = ptr_map_->at(map_index);
            // auto data = prob_map[map_index];

            if (data == 0) {
                save.data[map_index] = 255;
            }
            else if (data == 100) {
                save.data[map_index] = 0;
            }
            else {
                //   save.data[map_index] = data;
            }
        }
    }
    int idx = x1 + y1 * size_x;
    int idx2 = x1 + (y1 - 1) * size_x;
    save.data[idx] = 255;
    save.data[idx - 1] = 255;
    save.data[idx + 1] = 255;

    save.data[idx2] = 255;
    save.data[idx2 - 1] = 255;
    save.data[idx2 + 1] = 255;

    cv::flip(save, save, 0);
    static int count = 0;
    std::string str_save_path_ = "/home/kevin/temp/";
    // cv::imwrite(str_save_path_ + function_name + std::to_string(count++) + ".png", save);
    cv::imwrite(str_save_path_ + function_name + ".png", save);
}

bool MapPlanner::ExistObstaclePath(const NaviFra::Pos& o_robot_pos, const std::vector<Pos> vec_local_path, int* n_collision_idx)
{
    if (o_navi_map_ == nullptr)
        return false;
    int x;
    int y;
    int8_t n8_tmp_val;
    float f_map_resolution = f_res_;  // cm per pixel.
    float robot_size_x = 1.2;
    float robot_size_y = 1.2;
    int barrier_x = (robot_size_x / f_map_resolution) / 2;
    int barrier_y = (robot_size_y / f_map_resolution) / 2;
    std::lock_guard<std::mutex> lock(mtx_set_map_);
    std::shared_ptr<std::vector<int8_t>> ptr_map_ = std::make_shared<std::vector<int8_t>>(o_navi_map_->GetMap());

    int n_x_pixel = o_navi_map_->GetXpx();
    int n_y_pixel = o_navi_map_->GetYpx();

    int n_path_size = vec_local_path.size() - 1;
    int n_Obn_Cnt = 10;

    if (n_path_size < n_Obn_Cnt) {
        return false;
    }

    for (int n_path = n_Obn_Cnt; n_path < n_path_size; n_path++) {
        int nGridPathX = (vec_local_path[n_path].GetXm() / f_map_resolution) + n_x_pixel / 2;
        int nGridPathY = (vec_local_path[n_path].GetYm() / f_map_resolution) + n_y_pixel / 2;

        if (nGridPathX < 0 || nGridPathX >= n_x_pixel || nGridPathY < 0 || nGridPathY >= n_y_pixel) {
            // LOG_ERROR("$$$$CONTINUE");

            // LOG_ERROR("GRID XY PIX XY: %d %d %d %d", nGridPathX, nGridPathY, n_x_pixel, n_y_pixel);

            continue;
        }
        n8_tmp_val = ptr_map_->at(nGridPathX + nGridPathY * n_x_pixel);
        if (n8_tmp_val <= (110) && n8_tmp_val >= (100)) {
            if (n_collision_idx != nullptr)
                *n_collision_idx = n_path;
            // SaveMap2(n_x_pixel, n_y_pixel, "JUSTINFUNC", nGridPathX, nGridPathY);

            // LOG_ERROR("GRID XY PIX XY: %d %d %d %d", nGridPathX, nGridPathY, n_x_pixel, n_y_pixel);
            return true;
        }
    }
    if (n_collision_idx != nullptr)
        *n_collision_idx = -1;

    return false;
}

// bool MapPlanner::ExistObstaclePath(const NaviFra::Pos & o_robot_pos, const std::vector<Pos> vec_local_path )
//{
//    int x;
//    int y;
//    char c_tmp_val;
//    float f_map_resolution = f_res_; // cm per pixel.
//    float robot_size_x = 1.2;
//    float robot_size_y = 1.2;
//    int barrier_x = (robot_size_x / f_map_resolution) / 2;
//    int barrier_y = (robot_size_y / f_map_resolution) / 2;

//     std::shared_ptr<std::vector<char>> ptr_map_  = std::make_shared<std::vector<char>>(o_navi_map_->GetMap());
//     int n_x_pixel=o_navi_map_->GetXpx() ;
//    int n_y_pixel=o_navi_map_->GetYpx();

//  int n_path_size= vec_local_path.size()-1;
//  int n_Obn_Cnt = 10;

//	    if(n_path_size < n_Obn_Cnt)
//      {
//        return false;
//  }

// for(int n_path=n_Obn_Cnt; n_path<n_path_size; n_path++ )
// {
//     int nGridPathX = (vec_local_path[n_path].GetXm() - o_navi_map_->GetXOriginM() )/f_map_resolution ;
//    int nGridPathY = ( vec_local_path[n_path].GetYm() -o_navi_map_->GetYOriginM() )/f_map_resolution ;

//           for(int i=-barrier_x; i < barrier_x; i+=1)
//          {
//            for(int j=-barrier_y; j < barrier_y; j+=1)
//          {
//            x = nGridPathX + i;
//          y = nGridPathY + j;
//        if(x < 0 || x >= n_x_pixel || y < 0 || y >= n_y_pixel)
//      {
//
//                       return true;
//                }
//                  c_tmp_val = ptr_map_->at(x + y*n_x_pixel);
//              if(c_tmp_val >= static_cast<char>(100))
//            {
//              return true;
//        }
//  }
//}

//         }

//      return false;

//}

// bool MapPlanner::ExistObstaclePath(const NaviFra::Pos & o_robot_pos, const std::vector<Pos> vec_local_path )
// {
//     int x;
//     int y;
//     char c_tmp_val;
//     float f_map_resolution = f_res_; // cm per pixel.
//     float robot_size_x = 1.2;
//     float robot_size_y = 1.2;
//     int barrier_x = (robot_size_x / f_map_resolution) / 2;
//     int barrier_y = (robot_size_y / f_map_resolution) / 2;

//     std::shared_ptr<std::vector<char>> ptr_map_  = std::make_shared<std::vector<char>>(o_navi_map_->GetMap());
//     int n_x_pixel=o_navi_map_->GetXpx() ;
//     int n_y_pixel=o_navi_map_->GetYpx();

//     int n_path_size= vec_local_path.size()-1;
//     int n_Obn_Cnt = 10;

//     if(n_path_size < n_Obn_Cnt){return false;}

//     for(int n_path=n_Obn_Cnt; n_path<n_path_size; n_path++ )
//     {
//         int nGridPathX = (vec_local_path[n_path].GetXm() - o_navi_map_->GetXOriginM() )/f_map_resolution ;
//         int nGridPathY = ( vec_local_path[n_path].GetYm() -o_navi_map_->GetYOriginM() )/f_map_resolution ;

//         for(int i=-barrier_x; i < barrier_x; i+=1)
//         {
//             for(int j=-barrier_y; j < barrier_y; j+=1)
//             {
//                 x = nGridPathX + i;
//                 y = nGridPathY + j;
//                 if(x < 0 || x >= n_x_pixel || y < 0 || y >= n_y_pixel)
//                 {

//                     return true;
//                 }
//                 c_tmp_val = ptr_map_->at(x + y*n_x_pixel);
//                 if(c_tmp_val >= static_cast<char>(100))
//                 {
//                     return true;
//                 }
//             }
//         }
//      }

//     return false;

// }

void MapPlanner::GetNearGoalIndex(int& n_goal_px, int& n_goal_py)
{
    int n_cnt_round = 30;
    float f_check_res = 0.05f;
    int CHECK_PX = floor(f_check_res / o_navi_map_->getresolutionM());  // start 5cm
    static const int col = o_navi_map_->GetXpx(), row = o_navi_map_->GetYpx();
    int c_start = n_goal_px - CHECK_PX, c_end = n_goal_px + CHECK_PX;
    int r_start = n_goal_py - CHECK_PX, r_end = n_goal_py + CHECK_PX;
    c_start = max(0, c_start);
    r_start = max(0, r_start);
    c_end = min(col - 1, c_end);
    r_end = min(row - 1, r_end);
    bool b_find = false;
    std::shared_ptr<std::vector<int8_t>> ptr_map_ = std::make_shared<std::vector<int8_t>>(o_navi_map_->GetMap());
    int8_t n8_val;
    while (!b_find) {
        for (int c = c_start; c < c_end; c++) {
            for (int r = r_start; r < r_end; c++) {
                if (ptr_map_->size() - 1 < c + r * col) {
                    continue;
                }
                n8_val = ptr_map_->at(c + r * col);
                if (n8_val < (100)) {
                    b_find = true;
                    n_goal_px = c;
                    n_goal_py = r;
                }
            }
        }
        f_check_res += 0.05f;
        if (n_cnt_round == 0) {
            b_find = true;
            break;
        }

        // reset for new search
        CHECK_PX = floor(f_check_res / o_navi_map_->getresolutionM());
        c_start = n_goal_px - CHECK_PX, c_end = n_goal_px + CHECK_PX;
        r_start = n_goal_py - CHECK_PX, r_end = n_goal_py + CHECK_PX;
        c_start = min(max(0, c_start), col - 1);
        r_start = min(max(0, r_start), row - 1);
        c_end = min(max(0, c_end), col - 1);
        r_end = min(max(0, r_end), col - 1);
        n_cnt_round--;
    }
    return;
}

std::vector<Pos> MapPlanner::PlanPath(const NaviFra::Pos& o_goal_pos, const float& f_head_rad)
{
    if (o_navi_map_->GetMapPointer()->empty()) {
        LOG_WARNING("MAP EMPTY...");
        return vector<Pos>();
    }
    // 참고하세요
    // ros_map_msg_.info.origin.position.x = core_status_.f_robot_pos_x_m - n_width / 2 * f_resolution;
    // ros_map_msg_.info.origin.position.y = core_status_.f_robot_pos_y_m - n_height/ 2 * f_resolution;
    // Pos o_local_goal = o_goal_pos - o_robot_pos;
    // o_robot_pos = (0, 0)
    int n_start_x_pixel = o_navi_map_->GetXpx() / 2;
    int n_start_y_pixel = o_navi_map_->GetYpx() / 2;
    int n_target_x_pixel = o_goal_pos.GetXm() / o_navi_map_->getresolutionM() + n_start_x_pixel;  // - p_gmap_info.first / 2;
    int n_target_y_pixel = o_goal_pos.GetYm() / o_navi_map_->getresolutionM() + n_start_y_pixel;  // - p_gmap_info.second / 2;
    LOG_INFO("BEFORE | pX:%d, pY:%d\n", n_target_x_pixel, n_target_y_pixel);
    // if( n_target_x_pixel < 0 ) {n_target_x_pixel = 0;}
    // else if( n_target_x_pixel >= o_navi_map_->GetXpx() ) {n_target_x_pixel = o_navi_map_->GetXpx()-1;}
    // if( n_target_y_pixel < 0 ) {n_target_y_pixel = 0;}
    // else if( n_target_y_pixel >= o_navi_map_->GetYpx() ) {n_target_y_pixel = o_navi_map_->GetYpx()-1;}

    if (n_target_x_pixel < 0) {
        LOG_WARNING("goal pos out of local map... %d, %d", n_target_x_pixel, n_target_y_pixel);
        return vector<Pos>();
    }
    else if (n_target_x_pixel >= o_navi_map_->GetXpx()) {
        LOG_WARNING("goal pos out of local map... %d, %d", n_target_x_pixel, n_target_y_pixel);
        return vector<Pos>();
    }
    if (n_target_y_pixel < 0) {
        LOG_WARNING("goal pos out of local map... %d, %d", n_target_x_pixel, n_target_y_pixel);
        return vector<Pos>();
    }
    else if (n_target_y_pixel >= o_navi_map_->GetYpx()) {
        LOG_WARNING("goal pos out of local map... %d, %d", n_target_x_pixel, n_target_y_pixel);
        return vector<Pos>();
    }

    // std::shared_ptr<std::vector<char>> ptr_map  = std::make_shared<std::vector<char>>(o_navi_map_->GetMap());

    // if( ptr_map->at(n_target_x_pixel+o_navi_map_->GetXpx()*n_target_y_pixel) >= static_cast<char>(100) )
    // {
    //     GetNearGoalIndex(n_target_x_pixel, n_target_y_pixel);
    // }

    LOG_INFO("robot START pixel (x, y) = " ANSI_COLOR_RED "%d, %d\n" ANSI_COLOR_RESET, n_start_x_pixel, n_start_y_pixel);
    LOG_INFO("robot GOAL pixel (x, y) = " ANSI_COLOR_GREEN "%d, %d\n" ANSI_COLOR_RESET, n_target_x_pixel, n_target_y_pixel);
    std::chrono::steady_clock::time_point tp_detect_time_ = std::chrono::steady_clock::now();

    std::vector<NaviFra::Pos> vec_path2 = o_astar_algorithm_.FindPath(n_start_x_pixel, n_start_y_pixel, n_target_x_pixel, n_target_y_pixel);

    std::chrono::duration<double> sec_check1 = std::chrono::steady_clock::now() - tp_detect_time_;

    if (vec_path2.empty()) {
        LOG_ERROR("Can't find path");
        return std::vector<Pos>();
    }

    vec_path2[0].SetRad(f_head_rad);
    // origin back to normal
    for (Pos& o_temp_pos : vec_path2) {
        o_temp_pos.f_x_m_ = o_temp_pos.GetXm() * o_navi_map_->getresolutionM() + o_navi_map_->GetXOriginM();
        o_temp_pos.f_y_m_ = o_temp_pos.GetYm() * o_navi_map_->getresolutionM() + o_navi_map_->GetYOriginM();
    }
    if (true == b_path_smoothing_) {
        // o_path_smoothing_.SetParam(0.1, 10);
        o_path_smoothing_.SetParam(0.1, 50);

        tp_detect_time_ = std::chrono::steady_clock::now();
        std::vector<NaviFra::Pos> vec_path = o_path_smoothing_.smoothingPath(vec_path2);
        sec_check1 = std::chrono::steady_clock::now() - tp_detect_time_;

        if (vec_path.size() < 1)
            vec_path = vec_path2;
        // vec_path.push_back(o_goal_pos);
        // vec_path.back().SetType(o_goal_pos.GetType());
        // vec_path.back().SetDeg(o_goal_pos.GetDeg());
        // vec_path.emplace_back(o_goal_pos);

        return vec_path;
    }
    else {
        // vec_path2.emplace_back(o_goal_pos);

        return vec_path2;
    }
}

std::vector<Pos> MapPlanner::PlanPath(const NaviFra::Pos& o_robot_pos, const NaviFra::Pos& o_goal_pos)
{
    if (o_navi_map_ == nullptr)
        return std::vector<Pos>();
    // NaviFra::Pos o_goal_pos;
    // o_goal_pos.SetXm(o_goal_node.GetXm());
    // o_goal_pos.SetYm(o_goal_node.GetYm());
    // o_goal_pos.SetDeg(o_goal_node.GetDeg());
    LOG_INFO("robot pos x : %f, y : %f, deg : %f", o_robot_pos.GetXm(), o_robot_pos.GetYm(), o_robot_pos.GetDeg());
    LOG_INFO("goal pos x : %f, y : %f, deg : %f", o_goal_pos.GetXm(), o_goal_pos.GetYm(), o_goal_pos.GetDeg());

    int n_start_x_offset_pixel = o_navi_map_->GetXpx() / 2;
    int n_start_y_offset_pixel = o_navi_map_->GetYpx() / 2;

    int n_start_x_pixel = o_robot_pos.GetXm() / o_navi_map_->getresolutionM() + n_start_x_offset_pixel;
    int n_start_y_pixel = o_robot_pos.GetYm() / o_navi_map_->getresolutionM() + n_start_y_offset_pixel;

    int n_target_x_pixel = o_goal_pos.GetXm() / o_navi_map_->getresolutionM() + n_start_x_offset_pixel;  // - p_gmap_info.first / 2;
    int n_target_y_pixel = o_goal_pos.GetYm() / o_navi_map_->getresolutionM() + n_start_y_offset_pixel;  // - p_gmap_info.second / 2;

    std::vector<NaviFra::Pos> vec_path2 = o_astar_algorithm_.FindPath(n_start_x_pixel, n_start_y_pixel, n_target_x_pixel, n_target_y_pixel);

    // origin back to normal
    if (vec_path2.empty()) {
        LOG_WARNING("Can't find path");
        return vec_path2;
    }
    for (Pos& o_temp_pos : vec_path2) {
        o_temp_pos.f_x_m_ = o_temp_pos.f_x_m_ * o_navi_map_->getresolutionM() + o_navi_map_->GetXOriginM();
        o_temp_pos.f_y_m_ = o_temp_pos.f_y_m_ * o_navi_map_->getresolutionM() + o_navi_map_->GetYOriginM();
    }
    vec_path2[0].SetRad(o_robot_pos.GetRad());

    Pos tmp_goal_pos;
    tmp_goal_pos.SetXm(n_target_x_pixel * o_navi_map_->getresolutionM() + o_navi_map_->GetXOriginM());
    tmp_goal_pos.SetYm(n_target_y_pixel * o_navi_map_->getresolutionM() + o_navi_map_->GetYOriginM());
    tmp_goal_pos.SetDeg(o_goal_pos.GetDeg());

    if (true == b_path_smoothing_) {
        o_path_smoothing_.SetParam(0.1, 5);
        // o_path_smoothing_.SetParam(0.1, 40);

        std::vector<NaviFra::Pos> vec_path = o_path_smoothing_.smoothingPath(vec_path2);
        if (vec_path.size() < 1)
            vec_path = vec_path2;
        vec_path.push_back(tmp_goal_pos);
        // vec_path.back().SetType(o_goal_pos.GetType());
        // vec_path.back().SetDeg(o_goal_pos.GetDeg());
        return vec_path;
    }
    else {
        return vec_path2;
    }
    return vector<NaviFra::Pos>();
}
std::vector<Pos> MapPlanner::PlanPath(const vector<std::string>& vec_node_id)
{
    std::vector<NaviFra::Pos> vec_path;
    return vec_path;
}

std::vector<Pos> MapPlanner::PlanPath(const NaviFra::Pos& o_robot_pos, const std::string& str_node_id)
{
    LOG_INFO("robot pos x : %f, y : %f, deg : %f", o_robot_pos.GetXm(), o_robot_pos.GetYm(), o_robot_pos.GetDeg());
    LOG_INFO("goal id : %s", str_node_id.c_str());

    NaviFra::NaviNode o_goal_node;
    bool b_goal_find = false;
    for (auto& node : vec_topology_node_) {
        // if(node.GetNodeID() == str_node_id)
        if (node.GetID() == str_node_id)

        {
            b_goal_find = true;
            o_goal_node = node;
            break;
        }
    }
    if (!b_goal_find) {
        LOG_ERROR("Can't find goal!");
        return std::vector<Pos>();
    }

    NaviFra::Pos o_goal_pos;
    o_goal_pos.SetXm(o_goal_node.GetXm());
    o_goal_pos.SetYm(o_goal_node.GetYm());
    o_goal_pos.SetDeg(o_goal_node.GetDeg());
    std::vector<NaviFra::Pos> vec_path = PlanPath(o_robot_pos, o_goal_pos);

    vec_path.back().SetType(o_goal_node.GetType());
    vec_path.back().SetDeg(o_goal_node.GetDeg());
    vec_path.back().SetGoalNodeName(o_goal_node.GetName());
    vec_path.back().SetGoalNodeID(o_goal_node.GetID());
    vec_path.back().SetNowNodeName(o_goal_node.GetName());
    vec_path.back().SetNowNodeID(o_goal_node.GetID());
    vec_path.back().SetNextNodeName(o_goal_node.GetName());
    vec_path.back().SetNextNodeID(o_goal_node.GetID());

    return vec_path;
}

std::vector<NaviFra::NaviNode> MapPlanner::FindNeighborNodes(const NaviFra::Pos& o_robot_pos, const int& n_find_num)
{
    std::vector<NaviNode> vec_nodes_sorted_dist;
    vec_nodes_sorted_dist.clear();
    for (int j = 0; j < vec_topology_node_.size(); j++) {
        vec_topology_node_[j].SetDistM(
            hypot(vec_topology_node_[j].GetXm() - o_robot_pos.GetXm(), vec_topology_node_[j].GetYm() - o_robot_pos.GetYm()));
        vec_nodes_sorted_dist.push_back(vec_topology_node_[j]);
    }
    sort(vec_nodes_sorted_dist.begin(), vec_nodes_sorted_dist.end());

    std::vector<NaviFra::NaviNode> vec_nearest_nodes;
    vec_nearest_nodes.clear();
    for (int i = 0; i < vec_nodes_sorted_dist.size(); i++) {
        vec_nearest_nodes.push_back(vec_nodes_sorted_dist[i]);
        if (vec_nearest_nodes.size() >= n_find_num) {
            return vec_nearest_nodes;
        }
    }
    return vec_nearest_nodes;
}

// NaviFra::Pos ConvertGlob2Loc(const NaviFra::Pos &o_global_pos)
// {
//     float n_start_x_m = o_navi_map_->GetXm()/2;
//     float n_start_y_m = o_navi_map_->GetYm()/2;
//     NaviFra::Pos o_pos_target;
//     o_pos_target.SetXm(o_global_pos.GetXm() - n_start_x_m);
//     o_pos_target.SetYm(o_global_pos.GetYm() - n_start_y_m);
//     o_pos_target.SetDeg(o_global_pos.GetDeg());

//     return o_pos_target;
// }

NaviFra::Pos MapPlanner::GetLocalGoal(const NaviFra::Pos& o_robot_pos, const vector<Pos> vec_global_path)
{
    Pos o_local_goal;
    for (int i = 0; i < vec_global_path.size(); i++) {
        float f_robot_x = o_robot_pos.GetXm();
        float f_robot_y = o_robot_pos.GetXm();

        float f_goal_x = vec_global_path[i].GetXm();
        float f_goal_y = vec_global_path[i].GetYm();

        float f_delta_x = f_robot_x - f_goal_x;
        float f_delta_y = f_robot_y - f_goal_y;

        float dist = f_delta_x * f_delta_x + f_delta_y * f_delta_y;

        if (dist >= 9.0) {
            o_local_goal = vec_global_path[i];
        }
        break;
    }

    return o_local_goal;
}
bool MapPlanner::GetOnPath(const Pos& o_robot_pos)
{
    return false;
}

NaviFra::Pos MapPlanner::GetNodePoseByName(const string& node_name)
{
    NaviFra::Pos o_result;

    return o_result;
}

}  // namespace NaviFra
