import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

class TurtlebotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()

        self.person_detected = False  # 사람 검출 여부를 저장하는 변수

        # YOLOv5 모델 로드 (PyTorch 사용)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    def laser_callback(self, data):
        # 라이다 센서 데이터 콜백 함수
        ranges = data.ranges
        min_distance = min(ranges)
        if min_distance < 0.5:
            self.stop_robot()

    def image_callback(self, data):
        # 웹캠 또는 비디오 스트림 열기
        cap = cv2.VideoCapture(0)  # 웹캠 사용 (0번 카메라)

        while True:
            # 프레임 읽기
            ret, frame = cap.read()

            if not ret:
                break

            # OpenCV BGR 이미지를 RGB로 변환
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # 이미지를 YOLO 모델에 전달하여 객체 감지
            results = model(img)

            # 사람 클래스 레이블
            person_class_label = 'person'

            # 감지된 객체 정보 얻기
            for obj in results.xyxy[0]:
                label = results.names[int(obj[5])]
                confidence = obj[4].item()
                if label == person_class_label and confidence >= 0.3:  # 사람 클래스 레이블 및 일정한 신뢰도 임계값
                    x1, y1, x2, y2 = map(int, obj[:4])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} ({confidence:.2f})", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 2)
            # 결과 출력
            cv2.imshow('Person Detection', frame)

            if person_detected:
                self.speak_message("사람이 있습니다. 주의하세요!")

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = TurtlebotController()
    controller.run()
