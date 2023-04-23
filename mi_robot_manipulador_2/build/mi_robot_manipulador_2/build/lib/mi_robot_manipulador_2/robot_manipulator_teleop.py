import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import sys, termios, tty, select
import signal 

class RobotManipulatorTeleop(Node):
    def __init__(self, joint_speeds):
        super().__init__('robot_manipulator_teleop')
        self.publisher_position_ = self.create_publisher(Vector3, '/robot_manipulator_position', 10)
        self.publisher_goal_ = self.create_publisher(Vector3, '/robot_manipulator_goal', 10)
        self.publisher_zone_ = self.create_publisher(String, '/robot_manipulator_zone', 10)
        self.joint_speeds = joint_speeds
        self.get_logger().info('Robot Manipulator Teleop node initialized')

    def publish_robot_position(self, x, y, z):
        # Crear un mensaje Vector3 con la posición actual del end-effector
        position_msg = Vector3()
        position_msg.x = x
        position_msg.y = y
        position_msg.z = z
        # Publicar el mensaje en el tópico '/robot_manipulator_position'
        self.publisher_position_.publish(position_msg)

    def publish_robot_goal(self, x, y, z):
        # Crear un mensaje Vector3 con la posición deseada del end-effector
        goal_msg = Vector3()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        # Publicar el mensaje en el tópico '/robot_manipulator_goal'
        self.publisher_goal_.publish(goal_msg)

    def publish_robot_zone(self, zone):
        # Crear un mensaje String con el nombre de la zona
        zone_msg = String()
        zone_msg.data = zone
        # Publicar el mensaje en el tópico '/robot_manipulator_zone'
        self.publisher_zone_.publish(zone_msg)

    def get_input(self):
        # Leer la entrada del usuario sin esperar por un retorno de carro
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rclpy.spin_once(self)

        while True:
            rclpy.spin_once(self)
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                key = sys.stdin.read(1)
                if key in ['w', 's', 'a', 'd', 'q', 'e', 'x', 'p', 'r']:
                    print("Tecla Presionada:", key)
                    break
                else:
                    print("Tecla Deshabilitada")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def teleop_loop(self):
        rate = self.create_rate(10) # Frecuencia de publicación de mensajes
        x = 0.0
        y = 0.0
        z = 0.0

        while rclpy.ok():
            key = self.get_input()

            if key == 'w':
                x += self.joint_speeds[0]
            elif key == 's':
                x -= self.joint_speeds[0]
            elif key == 'a':
                y += self.joint_speeds[1]
            elif key == 'd':
                y -= self.joint_speeds[1]
            elif key == 'q':
                z += self.joint_speeds[2]
            elif key == 'e':
                z -= self.joint_speeds[2]
            elif key == 'r':
                self.publish_robot_position(x, y, z) # Publicar posición actual del robot
            elif key == 'g':
                self.publish_robot_goal(x, y, z) # Publicar posición deseada del robot
            elif key == 'z':
                zone = input("Ingrese el nombre de la zona: ")
                self.publish_robot_zone(zone) # Publicar nombre de la zona

            # Imprimir las coordenadas actuales del end-effector
            print("Coordenadas del end-effector (x, y, z):", x, y, z)

            rate.sleep()


    def run(self):
        self.get_logger().info('Robot Manipulator Teleop node started')
        self.get_logger().info('Use w, s, a, d, q, e to control the robot end-effector position')
        self.get_logger().info('Use x to reset the position')
        self.get_logger().info('Use p to set the zone to pick')
        self.get_logger().info('Use r to set the zone to place')
        self.get_logger().info('Press Ctrl+C to abort the application') # Agregar información al usuario
        self.teleop_loop()


def main(args=None):
    rclpy.init(args=args)
    joint_speeds = [0.1, 0.1, 0.1] # Velocidades de movimiento para cada eje
    robot_teleop = RobotManipulatorTeleop(joint_speeds)
    
    
    robot_teleop.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
