#!/usr/bin/env python3
import time
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
class TurtlebotController:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.person_detected)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()

        # YOLOv3 가중치 파일 경로, 설정 파일 경로, 클래스 파일 경로
        self.weights_path = 'yolov3.weights'
        self.config_path = 'yolov3.cfg'
        self.class_labels_path = 'coco.names'
        # YOLOv3 모델 로드
        self.net = cv2.dnn.readNet(self.weights_path, self.config_path)
        # 클래스 레이블 로드
        with open(self.class_labels_path, 'r') as f:
            self.classes = f.read().splitlines()

        rospy.init_node("webcam_viewer")

    def laser_callback(self, data):
        global person
        
        ranges = data.ranges
        min_distance = min(ranges)
        if min_distance < 0.5:
            self.stop_robot()
            if person == True:
                self.right_robot()
                time.sleep(3)
                self.forward_robot()
                # 음성 출력
                # 원래 진행경로로 돌아가야 할지 의문(유니티로 하는 것인지 아니면 여기 코드에서 해야할지)
        else:
            self.forward_robot()

            

    def person_detected(self, data):
        global person 
        person = False
        # 프레임 읽기
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # 객체 감지를 위한 전처리
        blob = cv2.dnn.blobFromImage(frame, 1 / 255, (640, 480), (0, 0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        # 객체 감지 수행
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        outs = self.net.forward(output_layers)
        # 객체 감지 결과 처리
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and self.classes[class_id] == 'person':  # Confidence 임계값 및 'person' 클래스 필터링
                    person = True
                    center_x = int(detection[0] * frame.shape[1])
                    center_y = int(detection[1] * frame.shape[0])
                    width = int(detection[2] * frame.shape[1])
                    height = int(detection[3] * frame.shape[0])
                    x = int(center_x - width / 2)
                    y = int(center_y - height / 2)
                    boxes.append([x, y, width, height])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        # Non-Maximum Suppression을 사용하여 겹치는 박스 제거
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        # 사람 객체 그리기
        for i in range(len(boxes)):
            if i in indexes:
                x, y, width, height = boxes[i]
                label = str(self.classes[class_ids[i]])
                color = (0, 255, 0)  # 사각형 색상 설정 (여기서는 초록색)
                cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        # 결과 출력
        cv2.imshow('Person Detection', frame)
        if cv2.waitKey(27) == ord('q'):
            return
        


    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    # 오른쪽에도 장애물 있으면 라이다로 알 수 있나
    def right_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 1.0
        self.cmd_vel_pub.publish(twist_msg)

    def left_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = -1.0
        self.cmd_vel_pub.publish(twist_msg)

    def forward_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def run(self):
       rospy.spin()


if __name__ == '__main__':
    controller = TurtlebotController()
    controller.run()