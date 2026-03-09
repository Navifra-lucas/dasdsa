#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
테스트 결과 평가 스크립트
로그에서 추출한 실제 값과 예상 값을 비교
"""

import numpy as np
import math

def quaternion_to_yaw(qz, qw):
    """Quaternion z, w를 yaw angle (degrees)로 변환"""
    yaw_rad = 2 * math.atan2(qz, qw)
    return math.degrees(yaw_rad)

# 테스트 케이스 정의
test_cases = [
    {
        "name": "Test Case 1: 정면 3.5m",
        "expected": {"x": 3.5, "y": 0.0, "z": 0.0, "deg": 0.0},
        "actual": {
            "x": 3.3739837398373984,
            "y": 0.0,
            "z": 0.0,
            "qz": 0.0,
            "qw": 1.0
        }
    },
    {
        "name": "Test Case 2: 정면 2.5m",
        "expected": {"x": 2.5, "y": 0.0, "z": 0.0, "deg": 0.0},
        "actual": {
            "x": 2.4475823705605477,
            "y": 0.0,
            "z": 0.0,
            "qz": 0.0,
            "qw": 1.0
        }
    },
    {
        "name": "Test Case 3: 정면 4.5m",
        "expected": {"x": 4.5, "y": 0.0, "z": 0.0, "deg": 0.0},
        "actual": {
            "x": 4.270833333333334,
            "y": 0.010677083333333335,
            "z": 0.0,
            "qz": -0.001249997070324325,
            "qw": 0.999999218753357
        }
    },
    {
        "name": "Test Case 4: 왼쪽 10도 3.5m",
        "expected": {"x": 3.5, "y": 3.5 * math.tan(math.radians(10)), "z": 0.0, "deg": -10.0},
        "actual": {
            "x": 3.3739837398373984,
            "y": 0.6073170731707317,
            "z": 0.0,
            "qz": -0.08892883679185483,
            "qw": 0.9960379822009037
        }
    },
    {
        "name": "Test Case 5: 오른쪽 10도 3.5m",
        "expected": {"x": 3.5, "y": -3.5 * math.tan(math.radians(10)), "z": 0.0, "deg": 10.0},
        "actual": {
            "x": 3.3739837398373984,
            "y": -0.5904471544715447,
            "z": 0.0,
            "qz": 0.08651454456996178,
            "qw": 0.9962505877427887
        }
    }
]

print("=" * 80)
print("테스트 결과 평가 리포트")
print("=" * 80)
print()

total_score = 0
max_score = 0

for i, test in enumerate(test_cases, 1):
    print(f"\n{test['name']}")
    print("-" * 80)
    
    # 실제 각도 계산
    actual_deg = quaternion_to_yaw(test['actual']['qz'], test['actual']['qw'])
    
    # 예상 y 값 계산 (Test Case 4, 5의 경우)
    if i == 4:  # 왼쪽 10도
        expected_y = test['expected']['x'] * math.tan(math.radians(10))
    elif i == 5:  # 오른쪽 10도
        expected_y = -test['expected']['x'] * math.tan(math.radians(10))
    else:
        expected_y = test['expected']['y']
    
    # 오차 계산
    x_error = abs(test['actual']['x'] - test['expected']['x'])
    x_error_percent = (x_error / test['expected']['x']) * 100
    
    y_error = abs(test['actual']['y'] - expected_y)
    if abs(expected_y) > 0.001:
        y_error_percent = (y_error / abs(expected_y)) * 100
    else:
        y_error_percent = abs(test['actual']['y']) * 100  # y가 0이어야 하는 경우
    
    z_error = abs(test['actual']['z'] - test['expected']['z'])
    
    deg_error = abs(actual_deg - test['expected']['deg'])
    
    # 출력
    print(f"  X (거리):")
    print(f"    예상: {test['expected']['x']:.3f} m")
    print(f"    실제: {test['actual']['x']:.3f} m")
    print(f"    오차: {x_error:.3f} m ({x_error_percent:.2f}%)")
    
    print(f"  Y (좌우):")
    print(f"    예상: {expected_y:.3f} m")
    print(f"    실제: {test['actual']['y']:.3f} m")
    print(f"    오차: {y_error:.3f} m ({y_error_percent:.2f}%)")
    
    print(f"  Z (높이):")
    print(f"    예상: {test['expected']['z']:.3f} m")
    print(f"    실제: {test['actual']['z']:.3f} m")
    print(f"    오차: {z_error:.3f} m")
    
    print(f"  각도 (deg):")
    print(f"    예상: {test['expected']['deg']:.2f} deg")
    print(f"    실제: {actual_deg:.2f} deg")
    print(f"    오차: {deg_error:.2f} deg")
    
    # 평가 점수 (각 항목별 25점 만점)
    x_score = max(0, 25 - (x_error_percent * 2))  # 1% 오차당 2점 감점
    y_score = max(0, 25 - (y_error_percent * 2))
    z_score = 25 if z_error < 0.01 else max(0, 25 - (z_error * 100))  # z는 0이어야 함
    deg_score = max(0, 25 - (deg_error * 2))  # 1도 오차당 2점 감점
    
    case_score = x_score + y_score + z_score + deg_score
    total_score += case_score
    max_score += 100
    
    print(f"\n  점수: {case_score:.1f}/100")
    print(f"    - X: {x_score:.1f}/25")
    print(f"    - Y: {y_score:.1f}/25")
    print(f"    - Z: {z_score:.1f}/25")
    print(f"    - 각도: {deg_score:.1f}/25")

print("\n" + "=" * 80)
print("전체 평가")
print("=" * 80)
print(f"총점: {total_score:.1f}/{max_score} ({total_score/max_score*100:.1f}%)")
print()

# 종합 평가
if total_score >= 450:
    grade = "우수 (A)"
elif total_score >= 400:
    grade = "양호 (B)"
elif total_score >= 350:
    grade = "보통 (C)"
else:
    grade = "개선 필요 (D)"

print(f"등급: {grade}")
print()

# 세부 분석
print("세부 분석:")
print("-" * 80)

# X (거리) 정확도
x_errors = []
for test in test_cases:
    x_error = abs(test['actual']['x'] - test['expected']['x']) / test['expected']['x'] * 100
    x_errors.append(x_error)
avg_x_error = np.mean(x_errors)
print(f"  거리 추정 평균 오차: {avg_x_error:.2f}%")

# Y (좌우) 정확도
y_errors = []
for i, test in enumerate(test_cases):
    if i == 3:  # 왼쪽 10도
        expected_y = test['expected']['x'] * math.tan(math.radians(10))
    elif i == 4:  # 오른쪽 10도
        expected_y = -test['expected']['x'] * math.tan(math.radians(10))
    else:
        expected_y = 0.0
    
    if abs(expected_y) > 0.001:
        y_error = abs(test['actual']['y'] - expected_y) / abs(expected_y) * 100
    else:
        y_error = abs(test['actual']['y']) * 100
    y_errors.append(y_error)
avg_y_error = np.mean(y_errors)
print(f"  좌우 위치 추정 평균 오차: {avg_y_error:.2f}%")

# Z (높이) 정확도
z_errors = [abs(test['actual']['z']) for test in test_cases]
avg_z_error = np.mean(z_errors)
print(f"  높이 오차 평균: {avg_z_error:.4f} m (예상: 0.0 m)")

# 각도 정확도
deg_errors = []
for test in test_cases:
    actual_deg = quaternion_to_yaw(test['actual']['qz'], test['actual']['qw'])
    deg_error = abs(actual_deg - test['expected']['deg'])
    deg_errors.append(deg_error)
avg_deg_error = np.mean(deg_errors)
print(f"  각도 추정 평균 오차: {avg_deg_error:.2f} deg")

print()
print("=" * 80)

