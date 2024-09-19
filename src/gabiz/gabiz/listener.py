import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        qos_profile = QoSProfile(depth=10,reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.substitution = self.create_subscription(String,'topico',self.listener_callback, qos_profile)


    def run(self):
        rclpy.spin(self)
    
    def listener_callback(self,msg):
        self.get_logger().info(f'Publicou: {msg.data}')

    def __del__(self):
        self.get_logger().info('AAAAAAAAAADEUS')

def main():
    rclpy.init()
    node = Listener()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()