import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import sys, termios, tty, select
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key, Listener

pasos = input("Cuantos pasos quiere que den todos los motores inicialmente: ")
pasos_fijos = True

class RobotManipulatorTeleop(Node):
    def __init__(self, joint_speeds):
        super().__init__('robot_manipulator_teleop')
        self.publisher_comands_ = self.create_publisher(Twist, '/robot_manipulator_comands', 10)
        self.publisher_position_ = self.create_publisher(Vector3, '/robot_manipulator_position', 10)
        self.publisher_goal_ = self.create_publisher(Vector3, '/robot_manipulator_goal', 10)
        self.publisher_zone_ = self.create_publisher(String, '/robot_manipulator_zone', 10)
        self.joint_speeds = joint_speeds
        self.selected_motor = None
        self.get_logger().info('Robot Manipulator Teleop node initialized')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.last_key = None


    def publish_robot_position(self, x, y, z):
        # Crear un mensaje Vector3 con la posición actual del end-effectorwp
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
        # Publicar el mensaje en el tópico '/robot_manipulator_zone'wwwss
        self.publisher_zone_.publish(zone_msg)

    def callback_position(self, position_msg):
        # Actualizar la posición actual del robot
        self.robot_position = (position_msg.x, position_msg.y, position_msg.z)
        # Imprimir la posición actual del robot
        print("Posición actual del robot: ({position_msg.x}, {position_msg.y}, {position_msg.z})")
 
 
    def on_press(self, key):
        global pasos
        comand = Twist()    
        try:      
            
            if key.char == "i": #Motor1 adelante
                comand.angular.x = float(pasos)
                print(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint1 adelanTe')
                

            elif key.char == "k": #Motor1 atras
                comand.angular.x = -float(pasos)
                print(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint1 atras')

            elif key.char == "o": #Motor2 adelante
                comand.angular.y = float(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint2 adelante')


            elif key.char == "l": #Motor2 adelante
                comand.angular.y = -float(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint2 atras')


            elif key.char == "p": #Motor3 gripper abrir
                comand.angular.z = -float(165)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint3 abrir gripper')


            elif key.char == ";": #Motor3 gripper cerrar
                comand.angular.z = float(165)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint3 cerrar gripper')


            elif key.char == "m": #Modificar el numero de pasos
                pasos = input("Cuantos grados desea moverse")
                print("Pasos fijos es " + pasos)
        except:
            print("Tecla deshabilitada")


    def on_release(self, key):

        self.get_logger().info('Stop')
  
   
    def timer_callback(self):
        with keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()               


def main(args=None):
    rclpy.init(args=args)
    joint_speeds = [0.1, 0.1, 0.1] # Velocidades de los joints en cada dirección
    teleop_node = RobotManipulatorTeleop(joint_speeds)
    rclpy.spin(teleop_node)
    #motor, steps, direction = teleop_node.get_input()
    
    


if __name__ == '__main__':
    main()


