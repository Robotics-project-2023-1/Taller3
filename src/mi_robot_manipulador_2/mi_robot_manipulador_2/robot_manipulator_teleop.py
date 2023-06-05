import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
from pynput import keyboard
from pynput.keyboard import Key, Listener
from math import cos
from math import sin
from math import pi
from time import time

x_final = 8
y_final = 8
z_final = 8
pasos = int(input("Cuantos pasos quiere que den todos los motores inicialmente: "))
pasos_fijos = True
inicial1 = 90
inicial2 = 90
inicial3 = 90
inicial4 = 90
arreglo = [2,0,0,0,0,0,0] #Selector, lineal, angular, serv1, serv2, serv3, serv4

class RobotManipulatorTeleop(Node):
    def __init__(self):
        super().__init__('robot_manipulator_teleop')
        self.publisher_comands_ = self.create_publisher(Int32MultiArray, '/robot_manipulator_comands', 10)
        self.publisher_position_ = self.create_publisher(Point, '/robot_manipulator_position', 10)
        self.get_logger().info('Robot Manipulator Teleop node initialized')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.last_key = None


    def directa(self, grados_movidos):  #RODRI PON AQUI TUS ECUACIONES
        #grados_movidos es [serv1, serv2, serv3] son los cambios en cada comando
        global x_final
        global y_final
        global z_final
        
        theta_base = grados_movidos[2]
        theta1 = grados_movidos[0]
        theta2 = grados_movidos[1]
        link1_length = 9  # Length of link 1
        link2_length = 12  # Length of link 2


        theta_base = np.radians(theta_base)
        theta1 = np.radians(theta1)
        theta2 = np.radians(theta2)

        # DH parameters
        d0 = 0
        a0 = 0
        alpha0 = theta_base

        d1 = link1_length
        a1 = 0
        alpha1 = 0

        d2 = 0
        a2 = link2_length
        alpha2 = 0

        # Homogeneous transformation matrices
        T0 = np.array([[np.cos(alpha0), -np.sin(alpha0), 0, a0],
                       [np.sin(alpha0), np.cos(alpha0), 0, 0],
                       [0, 0, 1, d0],
                       [0, 0, 0, 1]])

        T1 = np.array([[np.cos(theta1), -np.sin(theta1)*np.cos(alpha1), np.sin(theta1)*np.sin(alpha1), a1*np.cos(theta1)],
                       [np.sin(theta1), np.cos(theta1)*np.cos(alpha1), -np.cos(theta1)*np.sin(alpha1), a1*np.sin(theta1)],
                       [0, np.sin(alpha1), np.cos(alpha1), d1],
                       [0, 0, 0, 1]])

        T2 = np.array([[np.cos(theta2), -np.sin(theta2)*np.cos(alpha2), np.sin(theta2)*np.sin(alpha2), a2*np.cos(theta2)],
                       [np.sin(theta2), np.cos(theta2)*np.cos(alpha2), -np.cos(theta2)*np.sin(alpha2), a2*np.sin(theta2)],
                       [0, np.sin(alpha2), np.cos(alpha2), d2],
                       [0, 0, 0, 1]])

        # Forward kinematics
        T = np.dot(T0, np.dot(T1, T2))

        # Extract position and orientation
        position = T[:3, 3]
        orientation = T[:3, :3]
                
        x_final = position[0]
        y_final = position[1]
        z_final = position[2]
        
        coordenadas = Point() #INFORMACION EN CM PORQUE NO SE PUEDE USAR FLOAT
        coordenadas.x = float(x_final)
        coordenadas.y = float(y_final)
        coordenadas.z = float(z_final)
        print(position[0],position[1],position[2])
        x_final = x_final + position[0]
        y_final = y_final + position[1]
        z_final = z_final + position[2]
        self.publisher_position_.publish(coordenadas)
        self.get_logger().info('Coordenadas publicadas')

 
    def on_press(self, key):
        global arreglo
        global pasos
        global inicial1
        global inicial2
        global inicial3
        comand = Int32MultiArray()
        try:      
            
            if key.char == "y": #Motor1 adelante
                self.directa([pasos,0,0])
                arreglo[3] = inicial1 + pasos
                comand.data = arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint1 adelanTe')
                inicial1 = inicial1 + pasos
                
            elif key.char == "h": #Motor1 atras
                self.directa([-pasos,0,0])
                arreglo[3] = inicial1 -pasos
                comand.data = arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint1 atras')
                inicial1 = inicial1 - pasos

            elif key.char == "u": #Motor2 adelante
                self.directa([0,pasos,0])
                arreglo[4] = inicial2 + pasos
                comand.data = arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint2 adelante')
                inicial2 = inicial2 + pasos


            elif key.char == "j": #Motor2 adelante
                self.directa([0,-pasos, 0])
                arreglo[4] = inicial2-pasos
                comand.data = arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint2 atras')
                inicial2 = inicial2 - pasos


            elif key.char == "i": #Motor3 atras
                self.directa([0,0,pasos])
                arreglo[5] = inicial3 + pasos
                comand.data = arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint3 atras')
                inicial3 = inicial3 + pasos

            elif key.char == "k": #Motor3 adelante
                self.directa([0,0,-pasos])
                arreglo[5] = inicial3-pasos
                comand.data = arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint3 adelante')
                inicial3 = inicial3 - pasos

            elif key.char == "o": #Motor4 gripper abrir
                arreglo[6] = -165
                comand.data = arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint4 abrir gripper')


            elif key.char == "l": #Motor4 gripper cerraroom
                arreglo[6] = 165
                comand.data = arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint4 cerrar gripper')
                

            elif key.char == "m": #Modificar el numero de pasos
                pasos = int(input("Cuantos grados desea moverse"))
                print("Pasos fijos es " + pasos)
                    
        except Exception as e:
            print(e)
            print("Error")


    def on_release(self, key):
        self.get_logger().info('Stop')
  
   
    def timer_callback(self):
        with keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()               


def main(args=None):
    rclpy.init(args=args)
    teleop_node = RobotManipulatorTeleop()
    rclpy.spin(teleop_node)
    
if __name__ == '__main__':
    main()

