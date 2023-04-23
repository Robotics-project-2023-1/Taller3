import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.subscription = self.create_subscription(Vector3, '/robot_manipulador_goal', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info('Received goal: x=%f, y=%f, z=%f' % (msg.x, msg.y, msg.z))


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
