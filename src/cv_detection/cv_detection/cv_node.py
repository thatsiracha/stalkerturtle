import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from detection_interfaces.msg import Detection
import math

import cv_bridge as cvb
import cv2 as cv

from ultralytics import YOLO

class CvNode(Node):

    def __init__(self, bridge:cvb.CvBridge):
        super().__init__("cv_node")
        self.create_subscription(
            Image,
            "/image_raw", # TODO: Specify camera topic later
            self.cvCallback,
            10
        )
        self.detectionPub = self.create_publisher(
            Detection,
            "/cv_detection",
            10
        )
        self.bridge = bridge
        self.frame = None
        self.detectMsg = Detection()

    def setMsg(self, params):
        self.detectMsg.detected = params[0]
        self.detectMsg.direction = params[1]
        self.detectMsg.offset = params[2]
        self.detectMsg.framefilled = params[3]

    def cvCallback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "rgb8") #cvImg is an ndarray usable by openCV
        
        #TODO: CV code goes here
        model = YOLO("yolo-Weights/yolov8n.pt")
        classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]
        results = model(self.frame, stream=True)
        no_of_person = 0
        detected = False
        direction = 0
        offset = 0
        frame_filled = 0
        other_person = 0
        # coordinates
        for r in results:
            
            boxes = r.boxes

            for box in boxes:
                
                cls = int(box.cls[0])
                if (classNames[cls]=="person"):
                    # bounding box
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
                    print(x1,y1,x2,y2)

                    # put box in cam
                    cv.rectangle(self.frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

                    # confidence
                    confidence = math.ceil((box.conf[0]*100))/100
                    print("Confidence --->",confidence)

                    # class name
                    cls = int(box.cls[0])
                    if (classNames[cls] == "person"):
                        no_of_person =  no_of_person + 1
                        if(other_person == 0):
                            offset = (640/2 - (x1 + x2)/2)/(640)
                            if (offset > 0):
                                direction = 1
                            frame_filled = (y2-y1)/(480)
                            other_person = 1
                        else:
                            print("More than one person detected in same frame")

                    print("Class name -->", classNames[cls])

                    # object details
                    org = [x1, y1]
                    font = cv.FONT_HERSHEY_SIMPLEX
                    fontScale = 1
                    color = (255, 0, 0)
                    thickness = 2

                    cv.putText(self.frame, classNames[cls], org, font, fontScale, color, thickness)
        if (no_of_person == 1):
            detected = True
        
        self.setMsg(detected, direction, offset, frame_filled)

        self.detectionPub.publish(self.detectMsg)

        cv.imshow('Webcam', self.frame)
        


def main(args = None):
    rclpy.init(args=args)
    bridge = cvb.CvBridge()
    cvNode = CvNode(bridge)
    try:
        rclpy.spin(cvNode)
    except KeyboardInterrupt:
        cvNode.get_logger().info("KeyboardInterrupt")
    cvNode.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


