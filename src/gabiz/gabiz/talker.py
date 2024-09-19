import rclpy
from rclpy.node import Node

from std_msgs.msg import String





class Publisher(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'topico',10)

    def run(self):

        self.get_logger().info('Publicando meu topico a cada 1 segundo :p')
        timer = self.create_timer(1,self.talker_callback)

        rclpy.spin(self)

    def talker_callback(self):
        msg = String()
        msg.data = 'Mensagemm!!! :D'
        self.publisher.publish(msg)

    def __del__(self):
        self.get_logger().info('ADEUS')

def main():
    rclpy.init()
    node = Publisher()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main