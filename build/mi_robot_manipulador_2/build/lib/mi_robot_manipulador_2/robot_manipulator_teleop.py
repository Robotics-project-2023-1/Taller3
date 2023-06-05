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


pasos = int(input("Cuantos pasos quiere que den todos los motores inicialmente: "))
pasos_fijos = True


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
        x_final = 1*grados_movidos[0]
        y_final = 1*grados_movidos[1]
        z_final = 1*grados_movidos[2]
        
        coordenadas = Point() #INFORMACION EN CM PORQUE NO SE PUEDE USAR FLOAT
        coordenadas.x = float(x_final)
        coordenadas.y = float(y_final)
        coordenadas.z = float(z_final)
        self.publisher_position_.publish(coordenadas)
        self.get_logger().info('Coordenadas publicadas')

 
    def on_press(self, key):
        self.arreglo = [2,0,0,0,0,0,0] #Selector, lineal, angular, serv1, serv2, serv3, serv4
        global pasos
        comand = Int32MultiArray()
        try:      
            
            if key.char == "u": #Motor1 adelante
                self.directa([pasos,0,0])
                self.arreglo[3] = pasos
                comand.data = self.arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint1 adelanTe')
                
            elif key.char == "j": #Motor1 atras
                self.directa([-pasos,0,0])
                self.arreglo[3] = -pasos
                comand.data = self.arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint1 atras')

            elif key.char == "i": #Motor2 adelante
                self.directa([0,pasos,0])
                self.arreglo[4] = pasos
                comand.data = self.arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint2 adelante')


            elif key.char == "k": #Motor2 adelante
                self.directa([0,-pasos, 0])
                self.arreglo[4] = -pasos
                comand.data = self.arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint2 atras')


            elif key.char == "o": #Motor3 atras
                self.directa([0,0,pasos])
                self.arreglo[5] = pasos
                comand.data = self.arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint3 atras')

            elif key.char == "l": #Motor3 adelante
                self.directa([0,0,-pasos])
                self.arreglo[5] = -pasos
                comand.data = self.arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint3 adelante')


            elif key.char == "p": #Motor4 gripper abrir
                self.arreglo[6] = -165
                comand.data = self.arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint4 abrir gripper')


            elif key.char == ";": #Motor4 gripper cerraroom
                self.arreglo[6] = 165
                comand.data = self.arreglo
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint4 cerrar gripper')
                

            elif key.char == "m": #Modificar el numero de pasos
                pasos = input("Cuantos grados desea moverse")
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


