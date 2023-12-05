import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from detection_interfaces.msg import Detection

import cv_bridge as cvb
import cv2 as cv

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

    def cvCallback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "rgb8") #cvImg is an ndarray usable by openCV
        
        #TODO: CV code goes here
        #model = YOLO("yolo-Weights/yolov8n.pt")


def main(args = None):
    rclpy.init(args=args)
    bridge = cvb.CvBridge()
    cvNode = CvNode(bridge)
    try:
        rclpy.spin(cvNode)
    except KeyboardInterrupt:
        cvNode.get_logger().info("KeyboardInterrupt")
    cvNode.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


