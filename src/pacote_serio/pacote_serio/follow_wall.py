import rclpy
from rclpy.node import Node
import time 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import numpy

class FollowWall(Node):

    def __init__(self):
        super().__init__('follow_wall_node') #Define o nó

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile) #Subscriber Laser

        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile) #Odometria

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)#CMD_VEL

        self.integral = 0
        self.old_error = 0
        self.p_gain = 0.01 
        self.i_gain = 0.00 
        self.d_gain = 0.01 


    #Laser callback
    def listener_callback_laser(self, msg): 
        self.laser = msg.ranges

    #Laser callback
    def listener_callback_odom(self, msg): 
        self.pose = msg.pose.pose


    def PID(self):
        self.get_logger().info('PID')

        r_distance = numpy.array(self.laser[0:10]).mean() #Metade direita do laser (??)

        error = self.max_distance - r_distance
        self.integral = self.integral + error
        dif_error = error-self.old_error
        self.old_error = error 

        power = self.p_gain*error + self.i_gain*self.integral + self.d_gain*dif_error

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = 0.4, power
        self.pub_cmd_vel.publish(cmd)


    def run(self):

        while(1):
            rclpy.spin_once(self)

            self.max_distance = 2
            self.front_distance = 4

            f_laser = min((self.laser[ 80:100])) # -10 a  10 graus
            if f_laser <= self.front_distance:
                self.get_logger().info('CURVA')
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = 0.8, 2.0
                self.pub_cmd_vel.publish(cmd)
            else:
                self.PID()       

        
#Main Function
def main(): 
    rclpy.init()
    node = FollowWall()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  
