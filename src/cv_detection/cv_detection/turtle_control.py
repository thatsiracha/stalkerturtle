import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from detection_interfaces.msg import Detection

class ControllerGains:
    # Discrete-Time PI control
    def __init__(self, K = 1.0, Ti = 1.0, T = 0.0):
        self.kp = K*(1-T/(2*Ti))
        self.ki = K*T/Ti
        self.storedErr = 0
        self.storedOut = 0
        
    
    def getKp(self):
        return self.kp
    def getKi(self):
        return self.ki
    
    
    def controlLoop(self, fbval, tgtval):
        err = - fbval + tgtval
        term1 = (self.kp+self.ki)*err
        term2 = self.kp*self.storedErr
        term3 = self.storedOut
        out = term1-term2+term3
        if(abs(out) > 1):
            if (out > 0):
                out = 1
            else:
                out = -1

        self.storedErr = err
        self.storedOut = out
        return out


        

class ControlNode(Node):

    def __init__(self, controller_implemented=False, k = 1.0, ti = 5.0):
        super().__init__("turtle_control")
        timerT = 0.1;
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
        self.ctlimp = controller_implemented
        if(not controller_implemented):
            self.rot_threshold = 0.1
        else: 
            self.controller = ControllerGains(k,ti,timerT)

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
        angTgt = 0.0
        linTgt = 0.5

        if not self.det:
            return
            
            
        
        if abs(offset) > self.rot_threshold: 
            if self.ctlimp:
                angOut = self.controller.controlLoop(offset, angTgt)
                if rotation:
                    self.vel_cmd.angular.z = angVelMax*angOut
                else:
                    self.vel_cmd.angular.z = -1*angVelMax*angOut
            else:
                if rotation:
                    self.vel_cmd.angular.z = angVelMax*offset
                else:
                    self.vel_cmd.angular.z = -1*angVelMax*offset
        else:
            if self.ctlimp:
                linOut = self.controller.controlLoop(dist, linTgt)
                self.vel_cmd.linear.x = linOut*linVelMax
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

        

