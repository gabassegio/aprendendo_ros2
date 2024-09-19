import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

def main(args=None):
    # Inicializa o processo
    rclpy.init(args=args)
    
    # Controi o nó
    node = Node('no_tchau')

    # Algumas impressões e exemplos de uso do logger
    node.get_logger().info('Tchauuuu, nó 2 !!!')
   
    # Destroi o nó 
    node.destroy_node()

    # Finaliza o processo
    rclpy.shutdown()


if __name__ == '__main__':
    main()    




