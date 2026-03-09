# nc_intensity_detector 테스트 가이드

## 테스트 개요

가상의 인텐시티 이미지를 생성하여 반사 테이프 검출 기능을 테스트합니다.

## 사전 준비

1. **ROS 환경 설정**
```bash
source /opt/ros/noetic/setup.bash
source ~/navifra_solution/navicore/devel/setup.bash  # 또는 install/setup.bash
```

2. **ROS 마스터 실행**
```bash
roscore
```

## 테스트 방법

### 방법 1: 자동 테스트 (권장)

한 번에 모든 테스트를 실행합니다:

```bash
roslaunch nc_intensity_detector test_intensity_detector.launch
```

이 방법은 다음을 자동으로 실행합니다:
- nc_intensity_detector 노드
- 테스트 이미지 발행 노드
- 결과 모니터링

### 방법 2: 수동 테스트

여러 터미널에서 각각 실행:

**터미널 1: 검출 노드 실행**
```bash
rosrun nc_intensity_detector nc_intensity_detector_node
```

**터미널 2: 테스트 이미지 발행**
```bash
rosrun nc_intensity_detector test_intensity_publisher.py
```

**터미널 3: 결과 확인**
```bash
# 방법 A: 스크립트 사용
rosrun nc_intensity_detector check_detection_result.py

# 방법 B: 직접 확인
rostopic echo /nc_intensity_detector/pose
```

### 방법 3: 수동 제어

검출을 수동으로 시작/중지하면서 테스트:

```bash
# 검출 시작
rostopic pub /nc_intensity_detector/start std_msgs/Bool "data: true"

# 테스트 이미지 발행 (별도 터미널)
rosrun nc_intensity_detector test_intensity_publisher.py

# 검출 중지
rostopic pub /nc_intensity_detector/stop std_msgs/Bool "data: true"
```

## 테스트 케이스

테스트 스크립트는 다음 케이스를 자동으로 테스트합니다:

1. **정면 3.5m**: 기본 거리에서 정면 검출
2. **정면 2.5m**: 가까운 거리 검출
3. **정면 4.5m**: 먼 거리 검출
4. **왼쪽 10도 3.5m**: 왼쪽으로 치우친 테이프
5. **오른쪽 10도 3.5m**: 오른쪽으로 치우친 테이프
6. **정면 3.5m (30% 가려짐)**: 부분 가려짐 상황

## 예상 결과

각 테스트 케이스에서 다음 정보가 출력되어야 합니다:

```
Detection #1
  Position: x=3.500 m, y=0.000 m, z=0.000 m
  Orientation: yaw=0.00 deg
  Frame: base_link (또는 sick_visionary_t_mini)
  Timestamp: ...
```

### 검증 포인트

1. **거리 정확도**: x 좌표가 테스트 거리와 유사해야 함 (±0.5m 허용)
2. **각도 정확도**: y 좌표와 yaw가 테스트 각도와 유사해야 함
3. **가려짐 처리**: 30% 가려진 경우에도 검출되어야 함
4. **응답 시간**: 이미지 발행 후 1초 이내에 검출되어야 함

## 문제 해결

### 검출이 안 되는 경우

1. **검출이 활성화되었는지 확인**
```bash
rostopic echo /nc_intensity_detector/start
```

2. **인텐시티 임계값 확인**
```bash
rosparam get /nc_intensity_detector_node/intensity_threshold
```
기본값은 5000입니다. 테스트 이미지는 15000을 사용하므로 충분합니다.

3. **이미지가 발행되는지 확인**
```bash
rostopic hz /sick_visionary_t_mini/intensity
```

4. **노드 로그 확인**
검출 노드의 로그에서 다음을 확인:
- "Detection STARTED" 메시지
- "Tape detected" 메시지
- 에러 메시지

### TF 변환 오류

TF 변환이 실패하면 카메라 프레임에서 직접 포즈가 발행됩니다.
이 경우 `frames/base` 파라미터를 확인하거나 TF 트리를 설정하세요.

## 파라미터 조정

테스트 결과에 따라 `config/params.yaml`에서 다음을 조정할 수 있습니다:

- `intensity_threshold`: 인텐시티 임계값 (기본: 5000)
- `min_visible_ratio`: 최소 가시 비율 (기본: 0.5)
- `camera/focal_length_x`, `focal_length_y`: 카메라 초점 거리

## 테스트 이미지 커스터마이징

`test_intensity_publisher.py`를 수정하여 다른 테스트 케이스를 추가할 수 있습니다:

```python
test_cases = [
    {"name": "커스텀 테스트", "distance": 3.0, "angle": 0.0, "occluded": False},
    # 추가 테스트 케이스...
]
```

