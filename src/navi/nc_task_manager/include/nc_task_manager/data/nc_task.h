#ifndef NC_TASK_H
#define NC_TASK_H
namespace NaviFra {
class Task {
public:
    struct MoveData {
        std::string s_id = "";  // 노드ID
        std::string s_name = "";  // 노드이름
        int n_drive_type = 0;  // 노드 주행 타입 (1000: front, 1001: front diagonal, 2000: rear, 2001: rear diagonal)
        float f_speed_ms = 0.0f;  // 다음 노드까지 주행 속도
        float f_x_m = 0.0f;  // 노드 좌표 x
        float f_y_m = 0.0f;  // 노드 좌표 y
        float f_angle_deg = 0.0f;  // 노드 각도 deg
        float f_curve_radius = 0.0f;  // 세점에서 커브시 커브 반지름 (f_curve_radius, f_curvature 택1)
        float f_curvature = 0.0f;  // 두점에서 커브시 커브 곡률 (f_curve_radius, f_curvature 택1)
        int n_avoid_type =
            0;  // 회피 타입 (0: 회피 off, 1: 차선기반 회피(양쪽), 2: 차선기반 회피(왼쪽), 3: 차선기반 회피(오른쪽), 4: 맵기반 회피)
        float f_avoid_lanewidth = 0.0f;  // 회피 폭
        float f_avoid_speed_ms = 0.0f;  // 회피 속도
        float f_diagonal_heading_bias = 0.0f;  // 사행으로 갈때 강제로 heading bias 주는 옵션

        bool b_start_pause = false;  // 시작시 명령은 받았으나 pause상태 대기 옵션
        bool b_start_quick = false;  // 출발시 빠르게 출발하는 옵션
        bool b_stop_quick = false;  // 도착시 빠르게 정지하는 옵션

        std::vector<float> list_f_target_obstacle_margin = {
            0.0f, 0.0f, 0.0f, 0.0f};  // 도착지에 장애물 감지영역 임의 생성옵션 [front, rear, left, right] m단위
        std::vector<float> list_f_move_obstacle_margin = {
            0.0f, 0.0f, 0.0f, 0.0f};  // 도착지와 로봇사이 장애물 감지영역 임의 생성옵션 [front, rear, left, right] m단위

        bool b_arrive_align = false;  // 도착지에 도착 후 align 할지 말지 여부 default : false
        float f_arrive_boundary_dist = 0.0f;  // 도착지에 도착 후 거리상 크기에 따라서 알람
        float f_arrive_boundary_deg = 0.0f;  // 도착지에 도착 후 각도상 크기에 따라서 알람

        bool b_diagonal_align_skip = false;  // 사행 출발시 align 하지말고 바로 출발하는 옵션

        bool b_loaded = false;  // 적재 상태인지 구분명령
        bool b_lccs_off = false;

        std::vector<float> list_f_obstacle_margin = {-1.0f, -1.0f, -1.0f, -1.0f};  // 예측 감지 영역 크기 [front, rear, left, right]
        bool b_obstacle_side_check = false;  // 양 옆 side check 활성화 여부
        float f_side_check_margin_reduction_left = 0.0f;  // 양 옆 left 체크 영역
        float f_side_check_margin_reduction_right = 0.0f;  // 양 옆 right 체크 영역
    };

    struct ForkLiftData {
        std::string s_current_node_id = "";
        std::string s_target_node_id = "";

        float f_current_x = 0.0f;
        float f_current_y = 0.0f;
        float f_current_deg = 0.0f;
        float f_target_x = 0.0f;
        float f_target_y = 0.0f;
        float f_target_deg = 0.0f;

        int n_rack_level = 0;
        int n_target_level = 0;
        int n_target_height = 0;
        int n_drive_type = 0;
        int n_rack_type = 0;
        int n_pallet_type = 0;
    };

public:
    using Ptr = std::shared_ptr<Task>;

    Task();
    Task(Poco::JSON::Object::Ptr obj);
    ~Task();

public:
    std::string toString();
    Poco::JSON::Object toObject();
    const std::string& uuid() const;
    const std::string& type() const;
    const std::string& docking_type() const;
    const std::string& direction() const;
    double velocity();
    const std::string& start_node_name() const;
    const std::string& end_node_name() const;
    const std::string& startNode() const;
    const std::string& endNode() const;
    const std::string& targetNode() const;
    double finishAngle();
    bool isAlign();
    bool arrvieAlign();
    std::string print();
    const std::vector<MoveData>& moveData() const;
    const ForkLiftData& forkLiftData() const;
    std::vector<MoveData> vec_move_data_;
    ForkLiftData fork_lift_data_;
    Poco::JSON::Object::Ptr data() const { return data_; }
    int level();
    int count();
    int charger_id();
    int wait_seconds();
    std::string wait_id();
    int drive_type();
    float docking_dist();
    int pallet_exist();
    std::vector<float> goal_offset();

    Poco::JSON::Object::Ptr data_;

    std::string start_node_;
    std::string end_node_;
    std::string type_;
    std::string uuid_;
    std::string subtask_id_;
    std::string docking_type_;
    std::string direction_;
    std::string start_node_name_;
    std::string end_node_name_;
    std::string target_node_;

    std::vector<std::string> path_nodes_;

    bool is_align_;
    bool b_arrive_align_;

    float velocity_;
    float finish_angle_;
    float docking_dist_;

    std::vector<float> goal_offset_;
    float goal_offset_x_;
    float goal_offset_y_;
    float goal_offset_deg_;

    int level_;
    int count_;
    int charger_id_;
    int drive_type_;
    int pallet_exist_;

    const std::vector<float>& camera_roi() const;
    std::vector<float> list_f_camera_roi_;  // 도킹 카메라 roi영역 옵션 [y, z] m단위
    const std::vector<float>& camera_roi2() const;
    std::vector<float> list_f_camera_roi2_;  // 도킹 카메라 roi2영역 옵션 [y, z] m단위
    const std::vector<float>& move_obstacle() const;
    std::vector<float> list_f_move_obstacle_;  // 도킹 move obstacle
    const std::vector<float>& obstacle() const;
    std::vector<float> list_f_obstacle_;  // 도킹 move obstacle
    const std::vector<float>& target_obstacle() const;
    std::vector<float> list_f_target_obstacle_;  // 도킹 move obstacle
    // std::vector<float> list_f_move_obstacle_default_ = {0.0f, 0.0f, 0.0f, 0.0f}; 디폴트값 추가
    int wait_seconds_;
    std::string wait_id_;
    // 공용
};
}  // namespace NaviFra
#endif