import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv_bridge as cvb
import cv2 as cv

class CvNode(Node):

    def __init__(self, bridge:cvb.CvBridge):
        super.init("cv_node")
        self.create_subscription(
            Image,
            "/raspi/[TODO]", # TODO: Specify camera topic later
            self.cvCallback,
            10
        )
        self.bridge = bridge

    def cvCallback(self, msg):
        self.cvImg = self.bridge.imgmsg_to_cv2(msg) #cvImg is an ndarray usable by openCV
        
        #TODO: CV code goes here


def main(args):
    rclpy.init(args=args)
    bridge = cvb.CvBridge()
    cvNode = CvNode(bridge)
    rclpy.spin(cvNode)

    cvNode.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


