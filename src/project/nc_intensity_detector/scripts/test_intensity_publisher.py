#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
가상의 인텐시티 이미지를 발행하여 nc_intensity_detector 테스트
반사 테이프를 시뮬레이션하는 높은 인텐시티 값을 가진 직사각형 생성
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Bool

class IntensityTestPublisher:
    def __init__(self):
        rospy.init_node('intensity_test_publisher', anonymous=True)
        
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/sick_visionary_t_mini/intensity', Image, queue_size=1)
        self.start_pub = rospy.Publisher('/nc_intensity_detector/start', Bool, queue_size=1)
        self.stop_pub = rospy.Publisher('/nc_intensity_detector/stop', Bool, queue_size=1)
        
        # 이미지 파라미터 (SICK Visionary T Mini 실제 규격)
        self.image_width = 512
        self.image_height = 424
        
        # 테이프 파라미터 (실제 값과 동일)
        self.tape_width = 0.70  # meters (외부 프레임 가로)
        self.tape_height = 0.50  # meters (외부 프레임 세로)
        self.tape_thickness = 0.05  # meters (테이프 두께 5cm)
        
        # 카메라 파라미터
        # SICK Visionary T Mini: 화각 70도, 이미지 크기 512x424
        # FOV = 2 * arctan(sensor_size / (2 * focal_length))
        # 따라서: focal_length = sensor_size / (2 * tan(FOV / 2))
        fov_horizontal = np.radians(70.0)  # 수평 화각 70도
        # 수평 focal length 계산
        self.focal_length_x = (self.image_width / 2.0) / np.tan(fov_horizontal / 2.0)
        # 수직 focal length는 이미지 비율에 따라 계산 (같은 화각 비율 가정)
        # 또는 수직 화각도 70도라면:
        fov_vertical = np.radians(70.0)  # 수직 화각도 70도로 가정
        self.focal_length_y = (self.image_height / 2.0) / np.tan(fov_vertical / 2.0)
        self.principal_point_x = self.image_width / 2.0
        self.principal_point_y = self.image_height / 2.0
        
        rospy.loginfo("Camera parameters: FOV=70deg, focal_length=(%.1f, %.1f), image_size=(%d, %d)",
                     self.focal_length_x, self.focal_length_y, self.image_width, self.image_height)
        
        # 테스트 파라미터
        self.test_distance = 2.0  # meters (1.5m ~ 2.5m 사이)
        self.test_angle = 0.0  # radians (0 = 정면)
        self.intensity_value = 15000  # 높은 인텐시티 값 (0-65535)
        
        rospy.loginfo("Intensity Test Publisher initialized")
        rospy.loginfo("Image size: %dx%d", self.image_width, self.image_height)
        rospy.loginfo("Test distance: %.2f m", self.test_distance)
        
    def create_test_image(self, distance=None, angle=None, occluded=False):
        """
        테스트용 인텐시티 이미지 생성
        
        Args:
            distance: 테이프까지의 거리 (meters)
            angle: 테이프의 각도 (radians, 0 = 정면)
            occluded: 일부 가려짐 시뮬레이션 여부
        """
        if distance is None:
            distance = self.test_distance
        if angle is None:
            angle = self.test_angle
            
        # 배경 이미지 생성 (낮은 인텐시티)
        image = np.zeros((self.image_height, self.image_width), dtype=np.uint16)
        background_intensity = 1000
        image.fill(background_intensity)
        
        # 예상 픽셀 크기 계산
        # 공식: pixel_size = (real_size * focal_length) / distance
        # 각도가 있을 때는 투영 크기가 변하지만, 여기서는 정면 기준으로 계산
        # (실제로는 각도에 따라 크기가 변하지만, 테스트 목적상 단순화)
        expected_width_pixels = (self.tape_width * self.focal_length_x) / distance
        expected_height_pixels = (self.tape_height * self.focal_length_y) / distance
        thickness_pixels = (self.tape_thickness * self.focal_length_x) / distance  # 테이프 두께 (픽셀)
        
        # 가려짐 시뮬레이션
        if occluded:
            # 30% 가려짐 (높이의 일부만 보임)
            visible_height_ratio = 0.7
            expected_height_pixels *= visible_height_ratio
        
        # 테이프 중심 위치 계산
        # 각도에 따른 픽셀 오프셋: offset = tan(angle) * focal_length
        # (distance는 약분되어 제거됨)
        center_x = self.principal_point_x + (np.tan(angle) * self.focal_length_x)
        center_y = self.principal_point_y
        
        # 외부 직사각형 좌표 계산 (프레임 외곽)
        x1_outer = int(center_x - expected_width_pixels / 2)
        x2_outer = int(center_x + expected_width_pixels / 2)
        y1_outer = int(center_y - expected_height_pixels / 2)
        y2_outer = int(center_y + expected_height_pixels / 2)
        
        # 내부 직사각형 좌표 계산 (비워질 부분)
        x1_inner = int(x1_outer + thickness_pixels)
        x2_inner = int(x2_outer - thickness_pixels)
        y1_inner = int(y1_outer + thickness_pixels)
        y2_inner = int(y2_outer - thickness_pixels)
        
        # 이미지 범위 내로 제한
        x1_outer = max(0, min(self.image_width - 1, x1_outer))
        x2_outer = max(0, min(self.image_width - 1, x2_outer))
        y1_outer = max(0, min(self.image_height - 1, y1_outer))
        y2_outer = max(0, min(self.image_height - 1, y2_outer))
        
        x1_inner = max(0, min(self.image_width - 1, x1_inner))
        x2_inner = max(0, min(self.image_width - 1, x2_inner))
        y1_inner = max(0, min(self.image_height - 1, y1_inner))
        y2_inner = max(0, min(self.image_height - 1, y2_inner))
        
        # 외부 직사각형 그리기 (프레임 외곽 - 속이 꽉 참)
        cv2.rectangle(image, (x1_outer, y1_outer), (x2_outer, y2_outer), self.intensity_value, -1)
        
        # 내부 직사각형 지우기 (속을 비움 - 배경 인텐시티로 채움)
        if x1_inner < x2_inner and y1_inner < y2_inner:
            cv2.rectangle(image, (x1_inner, y1_inner), (x2_inner, y2_inner), background_intensity, -1)
        
        # 약간의 노이즈 추가 (현실감)
        noise = np.random.normal(0, 100, image.shape).astype(np.int16)
        image = np.clip(image.astype(np.int16) + noise, 0, 65535).astype(np.uint16)
        
        # 계산된 크기 정보 로그
        rospy.loginfo("Created test image: bbox=(%d,%d,%d,%d), thickness=%.1fpx, distance=%.2fm, angle=%.2fdeg, occluded=%s",
                     x1_outer, y1_outer, x2_outer-x1_outer, y2_outer-y1_outer, thickness_pixels, distance, np.degrees(angle), occluded)
        rospy.loginfo("  Calculated size: width=%.1fpx (expected %.1fpx), height=%.1fpx (expected %.1fpx), focal=(%.1f, %.1f)",
                     expected_width_pixels, expected_width_pixels, expected_height_pixels, expected_height_pixels,
                     self.focal_length_x, self.focal_length_y)
        
        return image
    
    def publish_image(self, image):
        """이미지를 ROS 토픽으로 발행"""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding="16UC1")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"  # 실제 센서 데이터와 동일한 frame_id
            self.image_pub.publish(msg)
        except Exception as e:
            rospy.logerr("Failed to publish image: %s", str(e))
    
    def run_tests(self):
        """여러 테스트 케이스 실행"""
        rospy.loginfo("Starting test sequence...")
        
        # 노드가 준비될 때까지 대기
        rospy.sleep(2.0)
        
        # 검출 시작
        rospy.loginfo("Sending START command...")
        start_msg = Bool()
        start_msg.data = True
        self.start_pub.publish(start_msg)
        rospy.sleep(1.0)
        
        test_cases = [
            {"name": "정면 1.5m", "distance": 1.5, "angle": 0.0, "occluded": False},
            {"name": "정면 2.0m", "distance": 2.0, "angle": 0.0, "occluded": False},
            {"name": "정면 2.5m", "distance": 2.5, "angle": 0.0, "occluded": False},
            {"name": "왼쪽 10도 2.0m", "distance": 2.0, "angle": np.radians(-10), "occluded": False},
            {"name": "오른쪽 10도 2.0m", "distance": 2.0, "angle": np.radians(10), "occluded": False},
            {"name": "정면 2.0m (30% 가려짐)", "distance": 2.0, "angle": 0.0, "occluded": True},
        ]
        
        for i, test_case in enumerate(test_cases):
            rospy.loginfo("\n=== Test Case %d: %s ===", i+1, test_case["name"])
            
            # Reset statistics for this test case
            reset_pub = rospy.Publisher('/nc_intensity_detector/reset_stats', Bool, queue_size=1)
            reset_msg = Bool()
            reset_msg.data = True
            reset_pub.publish(reset_msg)
            rospy.sleep(0.5)  # Wait for reset to complete
            
            # 테스트 이미지 생성 및 발행
            test_image = self.create_test_image(
                distance=test_case["distance"],
                angle=test_case["angle"],
                occluded=test_case["occluded"]
            )
            
            # 5초 동안 연속 발행
            rate = rospy.Rate(10)  # 10 Hz
            for _ in range(50):  # 5초
                self.publish_image(test_image)
                rate.sleep()
            
            rospy.loginfo("Test case %d completed. Waiting 2 seconds...", i+1)
            rospy.sleep(2.0)
        
        # 검출 중지
        rospy.loginfo("Sending STOP command...")
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_pub.publish(stop_msg)
        
        rospy.loginfo("All test cases completed!")

if __name__ == '__main__':
    try:
        publisher = IntensityTestPublisher()
        publisher.run_tests()
    except rospy.ROSInterruptException:
        pass

