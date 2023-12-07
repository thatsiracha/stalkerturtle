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
        self.count = 44
        self.side = True
        self.rot_threshold = 0.07
        
        
        if(controller_implemented):
            
            self.controller = ControllerGains(k,ti,1/30)

    def resetTwist(self):
        self.vel_cmd.angular.z = 0.0
        self.vel_cmd.linear.x = 0.0
        return

    def destroy_node(self):
        self.resetTwist()
        self.velPub.publish(self.vel_cmd);
        return super().destroy_node()
    
    def verbose(self, msg):
        self.get_logger().info(msg)
    
    def ctlCallback(self, msg):
        offset = msg.offset
        dist = msg.framefilled
        self.det = msg.detected
        rotation = msg.direction

        angVelMax = 1.6
        linVelMax = 0.20
        angTgt = 0.05
        linTgt = 0.9

        if not self.det:
            return
        self.resetTwist()
        if self.ctlimp:
            angOut = self.controller.controlLoop(offset, angTgt)
            linOut = self.controller.controlLoop(dist, linTgt)
            if abs(offset) > angTgt:
                if rotation:
                    self.vel_cmd.angular.z = -1*angVelMax*abs(angOut)
                    self.verbose("CONTROL TURN RIGHT: %f rad/s" % self.vel_cmd.angular.z)
                else:
                    self.vel_cmd.angular.z = angVelMax*abs(angOut)
                    self.verbose("CONTROL TURN LEFT: %f rad/s" % self.vel_cmd.angular.z)
            else:
                self.vel_cmd.linear.x = linOut*linVelMax
                self.fwd = max(self.fwd - 1, 0)
                if abs(self.vel_cmd.linear.x) < 0.01:
                        self.vel_cmd.linear.x = 0.0
                self.verbose("CONTROL FWD: %f m/s" % self.vel_cmd.linear.x)
            
        else:
            if abs(offset) > self.rot_threshold: 
                if rotation:
                    self.vel_cmd.angular.z = -1*1.1*angVelMax*offset
                    self.verbose("CONTROL TURN RIGHT: %f rad/s" % self.vel_cmd.angular.z)
                else:
                    self.vel_cmd.angular.z = 1.1*angVelMax*offset
                    self.verbose("CONTROL TURN LEFT: %f rad/s" % self.vel_cmd.angular.z)
            else:
                    self.vel_cmd.linear.x = linVelMax - linVelMax*(0.8*dist/linTgt)
                    if abs(self.vel_cmd.linear.x) < 0.01:
                        self.vel_cmd.linear.x = 0.0
                    self.fwd = max(self.fwd - 1, 0)
                    self.verbose("CONTROL FWD: %f m/s" % self.vel_cmd.linear.x)
        

        
    def neutralCall(self):
        if not self.det:
            self.resetTwist()
            
            if not self.fwd > self.lock:
                self.fwd += 1
                self.vel_cmd.linear.x = 0.07
                self.verbose("N: RUN FORWARD")
                
            else:
                if self.count == 0:
                    self.vel_cmd.angular.z = 0.6
                self.count += 1
                if self.count >= 45:
                    if self.side:
                        self.vel_cmd.angular.z = 0.6
                        self.side = not self.side
                        self.verbose("N: TURN LEFT")
                    else:
                        self.vel_cmd.angular.z = -1.2
                        self.side = not self.side
                        self.verbose("N: TURN RIGHT")
                        self.count = 0
                
                   
        self.velPub.publish(self.vel_cmd)
        

def main(args = None):
    rclpy.init(args=args)
    ctlNode = ControlNode(True, 1.5, 20)
    # ctlNode = ControlNode()
    rclpy.spin(ctlNode)

    ctlNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        

