from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from detection_interfaces.msg import Detection
from rclpy.parameter import Parameter

class DummyNode(Node):
    
    def __init__(self):
        super().__init__("dummy_node")
        self.dummy_pub = self.create_publisher(Detection, "/cv_detection", 10)
        T = 1/5
        self.dummyMsg = Detection()
        self.iter = 0;
        self.create_timer(T, self.dummyCall)
        self.series = {"forward_fast":[True, True, 0.02, 0.3], "right_fast":[True,True,0.3, 0.6], "left_slow":[True,False,0.15,0.7], "no_detect":[False,True,0.0,0.0]}

    def setMsg(self, params):
        self.dummyMsg.detected = params[0]
        self.dummyMsg.direction = params[1]
        self.dummyMsg.offset = params[2]
        self.dummyMsg.framefilled = params[3]

    def dummyCall(self):
        keys = ["forward_fast", "right_fast", "left_slow", "no_detect"]
        key = keys[self.iter]
        self.setMsg(self.series[key])
        self.dummy_pub.publish(self.dummyMsg)
        self.get_logger().info("Published " + key)
        self.iter += 1
        if self.iter > 3:
            self.iter =  0

def main(args = None):
    rclpy.init(args=args)
    dummyNd = DummyNode()
    rclpy.spin(dummyNd)

    dummyNd.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
    