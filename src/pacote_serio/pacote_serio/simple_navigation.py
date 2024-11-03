import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import tf_transformations
import numpy as np
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

OBJECTIVE = [8, 9]

class SimpleNavigation(Node):
    
    def __init__(self):
        super().__init__('simple_navigation_node')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.turn = True
        self.forward = False
        self.obstacle = False
    

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
       
    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def calculate_distance(self, coord=[0, 0]):
        pos = [self.pose.position.x, self.pose.position.y]
        self.distance = math.dist(pos, coord)
        self.orientation = math.atan2(coord[1] - pos[1], coord[0] - pos[0])

    def mesure_laser(self):
        ranges = np.array(self.laser)
        ranges[ranges == float('inf')] = 10  

        middle = len(ranges) // 2
        self.laser_right = np.min(ranges[:middle-20])
        self.laser_front = np.min(ranges[middle-20:middle+20])
        self.laser_left = np.min(ranges[middle+20:])

    def walk_along(self, cmd):
        if abs(self.theta) < 0.1:  
            cmd.linear.x = 0.0
            self.forward = False
            self.turn = True

        elif self.laser_front <= 2:
            cmd.linear.x = 0.0
            self.forward = False
            self.obstacle = True

        elif 1 <= self.distance <= 4:
            cmd.linear.x = 0.5

        elif self.distance < 0.5:
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0
            self.get_logger().info('Você chegou ao seu destino :D')
        self.pub_cmd_vel.publish(cmd)

    def avoid_obstacle(self, cmd):
        self.get_logger().info('Evitando Obstáculo')

        if self.laser_front > self.laser_right and self.laser_front > self.laser_left:
            cmd.angular.z = 0.0
        elif self.laser_left > self.laser_right:
            cmd.angular.z = 0.5
        elif self.laser_right > self.laser_left:
            cmd.angular.z = -0.5

        if self.laser_front > 1.5:
            cmd.linear.x = 0.5
        else:
            cmd.linear.x = 0.0

        self.pub_cmd_vel.publish(cmd)

    def turn_around(self, cmd):
        self.get_logger().info('Orientando')

        if abs(self.theta) >= 0.6: 
            cmd.angular.z = 0.5
            self.pub_cmd_vel.publish(cmd)
            self.get_logger().info('Tentando girar')



        else:
            cmd.angular.z = 0.0
            self.pub_cmd_vel.publish(cmd)
            self.get_logger().info(f'Orientado para {OBJECTIVE}')
            self.turn = False
            self.forward = True

        
    def run(self):
        rate = self.create_rate(10)  
        self.get_logger().info('Iniciando')
        rclpy.spin_once(self)
        
        while True:
            
            try:
                _ , _ , yaw = tf_transformations.euler_from_quaternion(
                    [self.pose.orientation.x, self.pose.orientation.y,
                    self.pose.orientation.z, self.pose.orientation.w])
            except:
                continue

            self.calculate_distance(OBJECTIVE)
            self.theta = self.orientation - yaw

            self.mesure_laser()
            rclpy.spin_once(self)
            cmd = Twist()

            if self.turn:
                self.turn_around(cmd)

            elif self.forward:
                self.get_logger().info('Seguindo em frente')
                self.walk_along(cmd)

            elif self.obstacle:
                self.get_logger().info('Evitando Obstáculo')
                self.avoid_obstacle(cmd)

            rate.sleep()

def main():
    rclpy.init()
    node = SimpleNavigation()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
