import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import sys, termios, tty, select

class RobotManipulatorTeleop(Node):
    def __init__(self, joint_speeds):
        super().__init__('robot_manipulator_teleop')
        self.publisher_position_ = self.create_publisher(Vector3, '/robot_manipulator_position', 10)
        self.publisher_goal_ = self.create_publisher(Vector3, '/robot_manipulator_goal', 10)
        self.publisher_zone_ = self.create_publisher(String, '/robot_manipulator_zone', 10)
        self.joint_speeds = joint_speeds
        self.selected_motor = None
        self.get_logger().info('Robot Manipulator Teleop node initialized')

    def publish_robot_position(self, x, y, z):
        # Crear un mensaje Vector3 con la posición actual del end-effector
        position_msg = Vector3()
        position_msg.x = x
        position_msg.y = y
        position_msg.z = z
        # Publicar el mensaje en el tópico '/robot_manipulator_position'
        self.publisher_position_.publish(position_msg)
        
        # Imprimir las coordenadas en la consola
        print("Posición actual del brazo robótico: ({x}, {y}, {z})")


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
        # Mostrar opciones de selección de motor
        print("Seleccione un motor (1, 2 o 3): ")
        motor = None
        while motor not in ['1', '2', '3']:
            motor = input()
        self.selected_motor = int(motor)

        # Obtener la cantidad de pasos
        print("Ingrese la cantidad de pasos (mayor a 0): ")
        steps = None
        while steps is None or steps <= 0:
            try:
                steps = int(input())
            except ValueError:
                print("Ingrese un número entero mayor a 0.")
        
        # Obtener la dirección de movimiento
        print("Seleccione la dirección de movimiento (-1, 0, 1): ")
        direction = None
        while direction not in ['-1', '0', '1']:
            direction = input()
        direction = int(direction)

        return self.selected_motor, steps, direction

    def callback_position(self, position_msg):
        # Actualizar la posición actual del robot
        self.robot_position = (position_msg.x, position_msg.y, position_msg.z)
        # Imprimir la posición actual del robot
        print("Posición actual del robot: ({position_msg.x}, {position_msg.y}, {position_msg.z})")



def main(args=None):
    rclpy.init(args=args)
    joint_speeds = [0.1, 0.1, 0.1] # Velocidades de los joints en cada dirección
    teleop_node = RobotManipulatorTeleop(joint_speeds)
    motor, steps, direction = teleop_node.get_input()

    # Suscribirse al tópico '/robot_manipulator_position' para recibir las posiciones publicadas
    subscription = teleop_node.create_subscription(Vector3, '/robot_manipulator_position', teleop_node.callback_position, 10)
    subscription  # Prevent unused variable warning

    # Mover el motor seleccionado en la dirección indicada
    if direction == -1:
        print("Moviendo motor", motor, "hacia atrás", steps, "pasos.")
        # Lógica para mover el motor hacia atrás
    elif direction == 0:
        print("No se mueve el motor", motor)
    elif direction == 1:
        print("Moviendo motor", motor, "hacia adelante", steps, "pasos.")
        # Lógica para mover el motor hacia adelante
    
    


if __name__ == '__main__':
    main()

"""
    def get_input(self):
        # Mostrar opciones de selección de motor
        print("Seleccione un motor (1, 2 o 3): ")
        motor = None
        while motor not in ['1', '2', '3']:
            motor = input()
        self.selected_motor = int(motor)

        # Obtener la cantidad de pasos
        print("Ingrese la cantidad de pasos (mayor a 0): ")
        steps = None
        while steps is None or steps <= 0:
            try:
                steps = int(input())
            except ValueError:
                print("Ingrese un número entero mayor a 0.")
        return self.selected_motor, steps

def main(args=None):
    rclpy.init(args=args)
    joint_speeds = [0.1, 0.1, 0.1] # Velocidades de los joints en cada dirección
    teleop_node = RobotManipulatorTeleop(joint_speeds)
    motor, steps = teleop_node.get_input()

if __name__ == '__main__':
    main()
"""