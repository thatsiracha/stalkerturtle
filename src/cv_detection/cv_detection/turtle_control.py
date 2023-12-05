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
        self.velPub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(timerT, self.neutralCall)
        neutral_fwd_limit = 10
        self.fwd = neutral_fwd_limit
        self.lock = neutral_fwd_limit
        self.det = 0
        self.vel_cmd = Twist()
        self.rot_threshold = 0.1

    def resetTwist(self):
        self.vel_cmd.angular.z = 0.0
        self.vel_cmd.linear.x = 0.0
        return

    def destroy_node(self):
        self.resetTwist()
        self.velPub.publish(self.vel_cmd);
        return super().destroy_node()
    
    def ctlCallback(self, msg):
        offset = msg.offset
        dist = msg.framefilled
        self.det = msg.detected
        rotation = msg.direction

        angVelMax = 1.2
        linVelMax = 0.2

        if not self.det:
            return
        if abs(offset) > self.rot_threshold: 
            if rotation:
                self.vel_cmd.angular.z = angVelMax*offset
            else:
                self.vel_cmd.angular.z = -1*angVelMax*offset
        else:
            self.vel_cmd.linear.x = linVelMax - linVelMax*dist
            self.fwd = max(self.fwd - 1, 0)
        

        
    def neutralCall(self):
        if not self.det:
            self.resetTwist()
            if not self.fwd > self.lock:
                self.fwd += 1
                self.vel_cmd.linear.x = 0.07
        else:
            self.det = False
        self.velPub.publish(self.vel_cmd)
        

def main(args = None):
    rclpy.init(args=args)
    ctlNode = ControlNode()
    rclpy.spin(ctlNode)

    ctlNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        

