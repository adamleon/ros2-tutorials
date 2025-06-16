import rclpy
from rclpy.node import Node

# This is just some node example to demonstrate the structure.
class SomeNode(Node):
    def __init__(self):
        super().__init__('some_node')
        self.get_logger().info('Node has started!')

# Initialize the node and spin it
def main(args=None):
    rclpy.init(args=args)
    node = SomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
