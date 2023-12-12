import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np


class TurtlebotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.ultrasonic_sub = rospy.Subscriber('/sensor_state', SensorState, self.ultrasonic_callback)
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()
        
        
    def laser_callback(self, data):
        # 라이다 센서 데이터 콜백 함수
        ranges = data.ranges
        min_distance = min(ranges)
        if min_distance < 0.5:
            self.stop_robot()
        
    def ultrasonic_callback(self, data):
        # 초음파 센서 데이터 콜백 함수
        distance = data.range
        if distance < 0.3:
            self.stop_robot()
        
    def image_callback(self, data):
        # YOLOv3 가중치 파일 경로, 설정 파일 경로, 클래스 파일 경로
        weights_path = 'yolov3.weights'
        config_path = 'yolov3.cfg'
        class_labels_path = 'coco.names'

        # YOLOv3 모델 로드
        net = cv2.dnn.readNet(weights_path, config_path)

        # 클래스 레이블 로드
        with open(class_labels_path, 'r') as f:
            classes = f.read().splitlines()

            # 비디오 캡처 객체 생성
            cap = cv2.VideoCapture(0)  # 0은 기본 웹캠

            while True:
                # 프레임 읽기
                ret, frame = cap.read()

           	   # 객체 감지를 위한 전처리
    	   blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), (0, 0, 0), True, crop=False)
       	   net.setInput(blob)

    	   # 객체 감지 수행
   	   layer_names = net.getLayerNames()
    	   output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    	   outs = net.forward(output_layers)

    	   # 객체 감지 결과 처리
    	   class_ids = []
    	   confidences = []
    	   boxes = []
    	   for out in outs:
       	       for detection in out:
            	           scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                            if confidence > 0.5 and classes[class_id] == 'person':  # Confidence 임계값 및 'person' 클래스 필터링
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
                label = str(classes[class_ids[i]])
                color = (0, 255, 0)  # 사각형 색상 설정 (여기서는 초록색)
                cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 결과 출력
        cv2.imshow('Person Detection', frame)


        if person_detected:
            self.speak_message("사람이 있습니다. 주의하세요!")
        
    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        
    def speak_message(self, message):
        pass
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = avoid_drive()
    controller.run()
