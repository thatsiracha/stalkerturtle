import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from detection_interfaces.msg import Detection
import math

import cv_bridge as cvb
import cv2 as cv

from ultralytics import YOLO

class CvNode(Node):

    def __init__(self, bridge:cvb.CvBridge):
        super().__init__("cv_node")
        self.create_subscription(
            CompressedImage,
            # Image,
            # "/image_raw", # TODO: Specify camera topic later
            "/image_raw/compressed",
            self.cvCallback,
            10
        )
        self.detectionPub = self.create_publisher(
            Detection,
            "/cv_detection",
            10
        )
        self.create_timer(0.1, self.display)
        self.bridge = bridge
        self.frame = None
        self.detectMsg = Detection()
        self.boxDrawn = False

    def setMsg(self, params):
        self.detectMsg.detected = params[0]
        self.detectMsg.direction = params[1]
        self.detectMsg.offset = params[2]
        self.detectMsg.framefilled = params[3]

    def cvCallback(self, msg):
        #self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8") #cvImg is an ndarray usable by openCV
        self.frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.boxDrawn = True
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
        direction = False
        offset = 0.0
        frame_filled = 0.0
        other_person = 0
        confidence_high = 0
        confidence_curr = 0
        # coordinates
        for r in results:
            
            boxes = r.boxes

            for box in boxes:
                
                cls = int(box.cls[0])

                #confidence
                confidence = math.ceil((box.conf[0]*100))/100
                print("Confidence --->",confidence)
                
                confidence_curr = confidence

                if (classNames[cls]=="person") and (confidence_curr > confidence_high):
                    confidence_high = confidence_curr
                    # bounding box
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
                    print(x1,y1,x2,y2)

                    # put box in cam
                    cv.rectangle(self.frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
                    

                    # class name
                    cls = int(box.cls[0])
                    if (classNames[cls] == "person"):
                        no_of_person =  no_of_person + 1
                        if(other_person == 0):
                            offset = (640/2 - (x1 + x2)/2)/(640)
                            if (offset > 0):
                                direction = True
                            else:
                                offset = abs(offset)

                            
                            frame_filled = abs((y2-y1)/(480))
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
            confidence_high = confidence_curr
        
        self.setMsg([detected, direction, offset, frame_filled])

        self.detectionPub.publish(self.detectMsg)

        
    def display(self):
        # if self.boxDrawn:
        if self.boxDrawn:
            cv.imshow('Webcam', self.frame)
            cv.waitKey(1)


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


