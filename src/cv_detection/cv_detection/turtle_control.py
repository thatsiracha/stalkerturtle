import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from detection_interfaces.msg import Detection

class ControlNode(Node):

    def __init__(self):
        super().__init__("turtle_control")
        timerT = 0.5;
        self.create_subscription(
            Detection,
            "/cv_detection",
            self.ctlCallback,
            10
        )
        self.velPub = self.create_publisher(Twist, "/cmd_vels")
        self.create_timer(timerT, self.neutralCall)
        self.fwd = 5
        self.lock = 5
        self.det = 0
        self.vel_cmd = Twist()
        self.rot_threshold = 0.05

    def ctlCallback(self, msg):
        offset = msg.offset
        dist = msg.framefilled
        self.det = msg.detected
        rotation = msg.direction

        somevalue = 1

        if not self.det:
            return
        if abs(offset) > self.rot_threshold: 
            if rotation:
                self.vel_cmd.angular.z = somevalue*offset
            else:
                self.vel_cmd.angular.z = -1*somevalue*offset
        else:
            self.vel_cmd.linear.x = somevalue*dist
        

        
    def neutralCall(self):
        if not self.det:
            vel_cmd = Twist()
            if not self.fwd > self.lock:
                self.fwd += 1
                self.vel_cmd.linear.x = 1
        self.velPub.publish(vel_cmd)


        

