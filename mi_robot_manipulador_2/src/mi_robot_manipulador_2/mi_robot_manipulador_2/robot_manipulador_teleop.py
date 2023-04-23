import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import sys, select, termios, tty


class TeleopRobot(Node):

    def __init__(self):
        super().__init__('teleop_robot')
        self.velocity = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.pub_goal = self.create_publisher(Vector3, '/robot_manipulador_goal', 10)
        self.pub_zone = self.create_publisher(String, '/robot_manipulador_zone', 10)
        self.sub_position = self.create_subscription(Vector3, '/robot_manipulador_position', self.position_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(sys.stdin.fileno())

        self.get_logger().info('Teleop robot initialized')

    def timer_callback(self):
        msg = Vector3()
        msg.x = self.velocity['joint1']
        msg.y = self.velocity['joint2']
        msg.z = self.velocity['joint3']
        self.pub_goal.publish(msg)

    def position_callback(self, msg):
        self.get_logger().info('Current position: x=%f, y=%f, z=%f' % (msg.x, msg.y, msg.z))

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                key = sys.stdin.read(1)
                if key == 'q':
                    break
                elif key == 'w':
                    self.velocity['joint1'] = min(1.0, self.velocity['joint1'] + 0.1)
                elif key == 's':
                    self.velocity['joint1'] = max(-1.0, self.velocity['joint1'] - 0.1)
                elif key == 'e':
                    self.velocity['joint2'] = min(1.0, self.velocity['joint2'] + 0.1)
                elif key == 'd':
                    self.velocity['joint2'] = max(-1.0, self.velocity['joint2'] - 0.1)
                elif key == 'r':
                    self.velocity['joint3'] = min(1.0, self.velocity['joint3'] + 0.1)
                elif key == 'f':
                    self.velocity['joint3'] = max(-1.0, self.velocity['joint3'] - 0.1)
                elif key == '1':
                    msg = String()
                    msg.data = 'zone1'
                    self.pub_zone.publish(msg)
                elif key == '2':
                    msg = String()
                    msg.data = 'zone2'
                    self.pub_zone.publish(msg)
                elif key == '3':
                    msg = String()
                    msg.data = 'zone3'
                    self.pub_zone.publish(msg)

        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        self.get_logger().info('Teleop robot stopped')


def main(args=None):
    rclpy.init(args=args)
    teleop_robot = TeleopRobot()
    teleop_robot.run()
    teleop_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()