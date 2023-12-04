import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from detection_interfaces.msg import Detection

class ControlNode(Node):

    def __init__(self):
        super().__init__("turtle_control")
        rate = self.create_rate(10.0);
        self.create_subscription(
            Detection,
            "/cv_detection",
            self.ctlCallback,
            10
        )

    def ctlCallback(self, msg):
        offset = msg.offset
        dist = msg.framefilled
