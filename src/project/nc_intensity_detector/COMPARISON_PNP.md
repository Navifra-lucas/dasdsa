# 현재 방식 vs PnP 방식 비교 분석

## 📋 개요

현재 구현된 방식과 제안된 PnP(Perspective-n-Point) 기반 방식을 상세히 비교 분석합니다.

---

## 🔍 현재 방식 (Edge-Line-Intersection Method)

### 프로세스
1. **영상 전처리**
   - Intensity thresholding (16-bit → 8-bit 변환)
   - Morphological operations (MORPH_CLOSE)로 프레임 연결
   - Canny edge detection

2. **특징점 추출**
   - HoughLinesP로 선분 검출
   - 선분을 수평/수직으로 클러스터링
   - 대표선(representative line) 계산 (가중 평균)
   - 선분 교차점으로 코너 계산 (4개)

3. **거리 추정**
   - 여러 방법 조합:
     - Contour bounding box 기반
     - Corner 기반 (테이프 두께 고려)
     - 대각선 거리 기반
   - 이상치 제거 (MAD 기반)
   - 가중 평균으로 최종 거리 계산

4. **자세 추정**
   - 기하학적 계산:
     - X = distance
     - Y = -distance * tan(angle)
     - Z = -(pixel_offset_y * distance) / focal_length_y
     - Yaw = atan2(pixel_offset_x, focal_length_x)

### 장점 ✅
1. **부분 가림 처리 우수**
   - 2개 코너만 있어도 동작 가능
   - 선분 기반으로 가려진 부분 추정 가능
   - 가림 비율 계산 및 거리 보정

2. **노이즈 강건성**
   - 이상치 선분 제거 (MAD 기반)
   - 여러 거리 추정 방법 조합
   - 가중 평균으로 안정성 향상

3. **프레임 구조 특화**
   - 중공 직사각형(테이프 프레임) 구조에 최적화
   - 내부/외부 경계 구분 (테이프 두께 고려)
   - 선분 기반으로 프레임 가장자리 검출

4. **유연한 검증**
   - 최소 2개 코너만 있어도 동작
   - 가림 상황에서도 거리 추정 가능
   - 선분 수 검증으로 신뢰도 판단

5. **실시간 성능**
   - 복잡한 최적화 없음
   - 직접적인 기하학적 계산
   - 빠른 처리 속도

### 단점 ❌
1. **서브픽셀 정밀도 부족**
   - 픽셀 단위 코너 위치
   - 각도 계산 시 미세 떨림 가능

2. **카메라 왜곡 미고려**
   - 현재 왜곡 보정 없음
   - 주변부에서 오차 증가 가능

3. **복잡한 로직**
   - 여러 단계의 추정 방법 조합
   - 파라미터 튜닝 복잡도 높음

4. **회전 자세 제한적**
   - Yaw만 계산 (Pitch, Roll 없음)
   - 테이프가 기울어진 경우 오차 가능

---

## 🎯 PnP 방식 (Perspective-n-Point)

### 프로세스
1. **영상 전처리**
   - Grayscale 변환
   - Thresholding (고정 임계값 또는 Otsu)
   - Morphology (선택적, 노이즈 제거)

2. **특징점 추출**
   - Contour finding (외곽선 검출)
   - approxPolyDP로 사각형 근사 (꼭짓점 4개)
   - **Corner SubPix**: 서브픽셀 정밀도 보정 ⭐
   - Corner ordering (3D 좌표 순서와 매칭)

3. **3D 자세 추정**
   - **cv2.solvePnP** 사용:
     - 입력: 3D 객체 좌표, 2D 이미지 좌표, Camera Matrix, Distortion
     - 출력: Translation Vector (x, y, z), Rotation Vector (rvec)
   - Rodrigues 변환으로 Pitch, Yaw, Roll 각도 계산

### 장점 ✅
1. **서브픽셀 정밀도**
   - Corner SubPix로 0.1도 단위 떨림 제거
   - 매우 정밀한 코너 위치 추정

2. **카메라 왜곡 보정**
   - Distortion coefficients 사용
   - 주변부에서도 정확한 측정

3. **완전한 자세 정보**
   - Translation (x, y, z)
   - Rotation (Pitch, Yaw, Roll)
   - 6-DOF 자세 추정

4. **검증된 알고리즘**
   - OpenCV 표준 함수 사용
   - 산업 표준 방법론

5. **단순한 파이프라인**
   - 전처리 → 특징점 → PnP
   - 명확한 단계 구분

### 단점 ❌
1. **4개 코너 필수**
   - 부분 가림 시 동작 어려움
   - 3개 이하 코너에서는 실패

2. **프레임 구조에 불리**
   - 중공 직사각형 검출 어려움
   - Contour가 내부/외부 경계를 혼동 가능
   - 테이프 두께 고려 어려움

3. **노이즈 민감**
   - 잘못된 contour 검출 시 전체 실패
   - Outlier 코너 처리 어려움

4. **카메라 캘리브레이션 필요**
   - Camera Matrix + Distortion Coefficients 필수
   - 현재는 focal length만 사용 중

5. **계산 비용**
   - solvePnP는 반복 최적화 수행
   - SubPix도 추가 계산 필요

---

## 📊 상세 비교표

| 항목 | 현재 방식 | PnP 방식 |
|------|----------|---------|
| **코너 정밀도** | 픽셀 단위 | 서브픽셀 (SubPix) |
| **부분 가림 처리** | ⭐⭐⭐⭐⭐ 우수 (2개 코너만 있어도 동작) | ⭐⭐ 낮음 (4개 코너 필수) |
| **프레임 구조 지원** | ⭐⭐⭐⭐⭐ 최적화됨 | ⭐⭐ 낮음 (중공 구조 어려움) |
| **노이즈 강건성** | ⭐⭐⭐⭐ 좋음 (이상치 제거) | ⭐⭐⭐ 보통 |
| **카메라 왜곡** | ❌ 미고려 | ✅ 보정 가능 |
| **자세 정보** | Yaw만 | 6-DOF (Pitch, Yaw, Roll) |
| **계산 속도** | ⭐⭐⭐⭐⭐ 빠름 | ⭐⭐⭐ 보통 |
| **구현 복잡도** | ⭐⭐⭐ 중간 | ⭐⭐ 낮음 |
| **파라미터 튜닝** | ⭐⭐ 복잡 | ⭐⭐⭐⭐ 단순 |
| **검증된 방법** | 커스텀 | ⭐⭐⭐⭐⭐ 산업 표준 |

---

## 🎯 사용 시나리오별 추천

### 현재 방식이 더 적합한 경우 ✅
1. **부분 가림이 빈번한 환경**
   - 로봇이 테이프를 부분적으로 가리는 경우
   - 2개 코너만 보이는 상황

2. **프레임 구조 (중공 직사각형)**
   - 테이프가 두께를 가진 프레임 형태
   - 내부/외부 경계 구분 필요

3. **실시간 성능 중요**
   - 빠른 처리 속도 필요
   - 계산 비용 최소화

4. **Yaw 각도만 필요**
   - Pitch, Roll 정보 불필요
   - 수평/수직 배치 가정

### PnP 방식이 더 적합한 경우 ✅
1. **고정밀도 요구**
   - 0.1도 단위 정밀도 필요
   - 서브픽셀 정확도 필수

2. **완전한 자세 정보 필요**
   - Pitch, Yaw, Roll 모두 필요
   - 6-DOF 자세 추정

3. **카메라 왜곡이 큰 경우**
   - 와이드 앵글 렌즈
   - 주변부 왜곡 보정 필요

4. **테이프가 항상 완전히 보이는 경우**
   - 가림 없는 환경
   - 4개 코너 항상 검출 가능

---

## 💡 하이브리드 접근법 제안

두 방식의 장점을 결합할 수 있습니다:

### 옵션 1: 조건부 선택
```cpp
if (detected_corners.size() == 4 && !is_occluded) {
    // PnP 방식 사용 (고정밀도)
    usePnP();
} else {
    // 현재 방식 사용 (강건성)
    useCurrentMethod();
}
```

### 옵션 2: 현재 방식에 SubPix 추가
- 현재의 선분 교차 방식 유지
- 코너 계산 후 Corner SubPix로 보정
- 서브픽셀 정밀도 향상

### 옵션 3: PnP에 프레임 구조 지원 추가
- Contour 검출 시 프레임 구조 고려
- 내부/외부 경계 구분
- 부분 가림 처리 로직 추가

---

## 🔧 현재 방식 개선 제안

PnP로 완전히 교체하기보다, 현재 방식을 개선하는 것이 더 실용적일 수 있습니다:

### 1. Corner SubPix 추가
```cpp
// 코너 계산 후 SubPix 보정
cv::cornerSubPix(intensity_image, corner_points, 
                 cv::Size(5, 5), cv::Size(-1, -1),
                 cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
```

### 2. 카메라 왜곡 보정 추가
```cpp
// SICK Visionary T Mini의 왜곡 계수 사용
cv::undistortPoints(corner_points, undistorted_points, 
                    camera_matrix, distortion_coeffs);
```

### 3. 거리 추정 개선
- 현재의 다중 방법 조합 유지
- PnP 결과와 비교하여 검증

---

## 📝 결론 및 권장사항

### 현재 방식 유지 + 개선 추천 ⭐
1. **부분 가림 처리**가 핵심 요구사항인 경우가 많음
2. **프레임 구조**에 최적화되어 있음
3. **실시간 성능**이 중요함
4. **Yaw 각도만** 필요할 가능성이 높음

### 개선 방향
1. ✅ **Corner SubPix 추가**: 정밀도 향상
2. ✅ **카메라 왜곡 보정**: 주변부 정확도 향상
3. ✅ **PnP 결과와 비교**: 검증 및 보정

### PnP 방식 도입 고려 시점
- 부분 가림이 거의 없는 환경
- 6-DOF 자세 정보가 필요한 경우
- 고정밀도가 최우선 요구사항인 경우

---

## 📚 참고사항

### 현재 구현의 강점
- **가림 처리**: `occlusion_distance_penalty`, `min_visible_ratio` 파라미터
- **노이즈 필터링**: `removeOutlierLines` (MAD 기반)
- **다중 거리 추정**: 3가지 방법 조합
- **프레임 두께 고려**: `tape_thickness` 반영

### PnP 구현 시 필요사항
- Camera Matrix (K): `fx, fy, cx, cy`
- Distortion Coefficients (D): `k1, k2, p1, p2, k3`
- 3D 객체 좌표 정의: 테이프 중심 기준 4개 점
- Corner ordering: 3D 좌표와 2D 좌표 매칭

