# 팔레트 인식 테스트 순서

## 1. 실행 순서

### 터미널 1: roscore
```bash
roscore
```

### 터미널 2: 포인트 클라우드 소스
SICK Visionary T Mini 등 depth 카메라 드라이버를 띄워 `/sick_visionary_t_mini/points` (또는 perception.json의 `depth_camera_1` 토픽)가 발행되도록 합니다.

- 실제 센서: 해당 카메라 드라이버 노드 실행
- 재생 테스트: `rosbag play your_bag.bag`

### 터미널 3: Perception 노드 (팔레트 인식)
```bash
# kia_poc 설정 사용 (config/robot/kia_poc/ 사용)
rosrun answer perception_node _robot_name:=kia_poc
```

- `_robot_name` 생략 시 `config/default/` 사용
- 노드가 올라오면 `pallet_detector.json`의 `template_lists`로 템플릿(jong, plastic, wood 등)을 로드합니다.

### 터미널 4: 인식 시작 명령
팔레트 타입과 **예상 거리(x)**를 지정해 `/perception/cmd`로 JSON을 보냅니다.  
**x는 로봇 기준 팔레트까지 예상 거리 [m]이며, 2.5m보다 커야 인식이 동작합니다.** (예: 3.7m 앞이면 x=3.7)

```bash
# x=예상거리(m, 2.5 초과 권장), type: 0=type1, 1=type2, 2=jong, 3=plastic, 4=wood
rosrun answer rostopic pub -1 /perception/cmd std_msgs/String "data: '{\"cmd\":\"start\",\"dist\":2.5,\"height\":1.0,\"width\":1.0,\"type\":2,\"x\":3.7,\"y\":0,\"z\":0,\"deg\":0,\"local\":true}'"
```

- **x** → 로봇~팔레트 예상 거리 [m] (2.5보다 크게 설정해야 인식 실행)
- **type=2** → jong  
- **type=3** → plastic  
- **type=4** → wood  

인식 중지:
```bash
rosrun answer rostopic pub -1 /perception/cmd std_msgs/String "data: '{\"cmd\":\"stop\"}'"
```

---

## 2. 요약

| 순서 | 실행 내용 |
|------|-----------|
| 1 | `roscore` |
| 2 | 포인트 클라우드 발행 (드라이버 또는 rosbag) |
| 3 | `rosrun answer perception_node _robot_name:=kia_poc` |
| 4 | `/perception/cmd`로 `cmd: start`, `type: 2/3/4` 등 전송 |

템플릿 파일(`jong.ply`, `plastic.ply`, `wood.ply`)은 `answer` 패키지의 **template/** 디렉터리에 있어야 하며, `pallet_detector.json`의 `template_lists` 순서와 `type` 인덱스가 일치해야 합니다.

---

## 3. 인식 디버그 토픽 (RViz)

인식이 실행될 때마다 아래 토픽으로 **1회 발행**되며(latched), RViz에서 매칭 위치를 확인할 수 있습니다.

| 토픽 | 설명 |
|------|------|
| `/perception/debug/template_initial` | 초기 추정 위치 템플릿 (매 프레임 + 콜백) |
| `/perception/debug/template_aligned` | ICP 매칭 결과로 정렬된 템플릿 (매칭 완료 시 갱신) |
| `/perception/debug/sensor_raw` | 센서 원본 포인트클라우드 (후처리 전, 매 프레임) |
| `/perception/debug/sensor_matched` | 후처리 끝난 센서 = ICP 매칭에 사용된 포인트클라우드 |

- **Frame ID**: `base_link`
- RViz에서 `By topic` → 위 토픽 추가 후, Fixed Frame을 `base_link`로 맞추면 됩니다.
