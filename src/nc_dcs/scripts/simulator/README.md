# nc_dcs Simulator Package

nc_dcs 노드와 통신 테스트를 위한 간단한 시뮬레이터 패키지입니다.

## 구성

### 1. fork_simulator.py
포크리프트 제어 시뮬레이터
- **Subscribe**: `/forkinfo` (WiaForkInfo)
- **Publish**: `/cheonil/read_register` (CheonilReadRegister)
- **기능**: 포크 높이, 너비, 틸트 제어 시뮬레이션

### 2. path_plan_simulator.py
경로계획 및 도킹 시뮬레이터
- **Subscribe**: `/path_plan/cmd` (PoseStamped), `/path_plan/goback` (Bool)
- **Publish**: `/path_plan/status` (String)
- **기능**: 도킹 및 후진 동작 시뮬레이션

### 3. perception_simulator.py
인식 시뮬레이터
- **Subscribe**: `/perception/cmd` (String)
- **Publish**: `/perception/pose` (PoseStamped)
- **기능**: 팔레트/랙 위치 인식 시뮬레이션

### 4. test_launcher.py
테스트 미션 발송 스크립트
- **Publish**: `/nc_task_manager/fork_docking` (ForkLift)
- **기능**: nc_dcs에 테스트 미션 전송

## 사용 방법

### 방법 1: 개별 실행 (디버깅용)

터미널 1 - Fork Simulator:
```bash
cd ~/navifra_solution/navicore
source devel/setup.bash
rosrun nc_dcs fork_simulator.py
```

터미널 2 - Path Plan Simulator:
```bash
source devel/setup.bash
rosrun nc_dcs path_plan_simulator.py
```

터미널 3 - Perception Simulator:
```bash
source devel/setup.bash
rosrun nc_dcs perception_simulator.py
```

터미널 4 - nc_dcs 노드:
```bash
source devel/setup.bash
rosrun nc_dcs docking_control_node
```

터미널 5 - 테스트 미션 전송:
```bash
source devel/setup.bash
rosrun nc_dcs test_launcher.py
```

### 방법 2: 일괄 실행 스크립트

```bash
cd ~/navifra_solution/navicore/src/nc_dcs/scripts/simulator
./run_all_simulators.sh
```

## 수동 테스트

### 포크 제어 테스트
```bash
# 포크 높이 1500mm로 상승
rostopic pub /forkinfo core_msgs/WiaForkInfo "n_fork_height: 1500
n_fork_wide: -1
f_fork_tilt: -1.0
b_cancel: false"

# 포크 너비 넓게
rostopic pub /forkinfo core_msgs/WiaForkInfo "n_fork_height: -1
n_fork_wide: 1
f_fork_tilt: -1.0
b_cancel: false"
```

### 인식 시작
```bash
rostopic pub /perception/cmd std_msgs/String "data: 'start'"
```

### 도킹 명령
```bash
rostopic pub /path_plan/cmd geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position: {x: 2.0, y: 1.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

### 후진 명령
```bash
rostopic pub /path_plan/goback std_msgs/Bool "data: true"
```

## 토픽 모니터링

```bash
# 모든 토픽 확인
rostopic list

# 포크 상태 모니터링
rostopic echo /cheonil/read_register

# 경로 계획 상태 모니터링
rostopic echo /path_plan/status

# 인식 결과 모니터링
rostopic echo /perception/pose

# nc_dcs 로그 확인
rosnode info /docking_control_node
```

## 테스트 시나리오

1. **팔레트 픽업 시퀀스 (drive_type=1)**
   - Perception 활성화 → 팔레트 인식
   - 도킹 수행
   - 포크 하강
   - 포크 상승 (팔레트 들어올림)
   - 후진

2. **랙 적재 시퀀스 (drive_type=2)**
   - 포크 상승 (목표 높이)
   - 전진 (도킹)
   - 포크 하강 (적재)
   - 후진

## 트러블슈팅

### 시뮬레이터가 메시지를 받지 못하는 경우
```bash
# ROS 마스터 확인
roscore

# 토픽 연결 확인
rostopic info /forkinfo
rostopic info /perception/cmd
rostopic info /path_plan/cmd
```

### nc_dcs가 시퀀스를 실행하지 않는 경우
- sequences.yaml 파일이 제대로 로드되었는지 확인
- drive_type이 설정 파일에 정의되어 있는지 확인
- 로그에서 에러 메시지 확인

## 파일 권한 설정

```bash
chmod +x *.py
chmod +x *.sh
```
