import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import sys, termios, tty, select
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key, Listener
from math import cos
from math import sin
from math import pi
from time import time


pasos = input("Cuantos pasos quiere que den todos los motores inicialmente: ")
entradas = input("Coordenadas: ").split(",")
pasos_fijos = True

TOL=1e-6 # error cuadrático de respuestaq
timeout=1 # timeout de 1 segundo


r1=3.5
r2=8
r3=8
r4=3.5
r5=8
A=[0,0]
D=[0,0]

#Ef=[] : coordenads [x,y] del end-effector
#Servos=[]:  angulos [theta1, theta3] correspondientes a los servos
#Xa=np.array([[],[],[],[]]): arreglo np donde [[theta1],[theta2],[theta3],[theta4]] correspnden a la configuración actual 


def Newton_Rhapson(J,F,X0, TOL):
    global timeout
    tstart=time()
    tfin=time()
    Xp=X0
    X= Xp-np.linalg.inv(J(Xp)).dot(F(Xp))
    while np.linalg.norm(X-Xp)>TOL and (tfin-tstart)<=timeout:
        Xp=X
        X-=np.linalg.inv(J(X)).dot(F(X))
        tfin=time()
    if (tfin-tstart)<=timeout:
        res=X
    else:
        res=np.ones([len(X0),len(X0[0])])*np.nan
    return res


def directa(Servos):
    global TOL
    global r1
    global r2
    global r3
    global r4
    global r5
    global A
    global D
    
    Servos= [Servos[0]*(pi/180), Servos[1]*(pi/180)]
    
    X0=np.array([[pi/2],[0],[8],[8]])
    Fdir=lambda X: np.array([A[0]+r1*cos(Servos[0])+r2*cos(X[0])+(r4+r5)*cos(X[1])-X[2],
                          D[0]+r3*cos(Servos[1])+r5*cos(X[1])-X[2],
                          A[1]+r1*sin(Servos[0])+r2*sin(X[0])+(r4+r5)*sin(X[1])-X[3],
                          D[1]+r3*sin(Servos[1])+r5*sin(X[1])-X[3]])
    
        
    Jdir=lambda X: np.array([[-r2*sin(X[0]), -(r4+r5)*sin(X[1]),-1,0],
                            [0, -r5*sin(X[1]),-1,0],
                            [r2*cos(X[0]), (r4+r5)*cos(X[1]),0,-1],
                            [0, r5*cos(X[1]),0,-1]])
    
    X=Newton_Rhapson(Jdir,Fdir,X0, TOL)
    
    return np.array([[X[0]*(180/pi)],[X[1]*(180/pi)],[X[2]],[X[3]]])


def inversa(x_goal, y_goal, z_goal, Xa): #Xa es la configuracion actual de angulos
    global TOL
    global r1
    global r2
    global r3
    global r4
    global r5
    global A
    global D
    
    Xa=Xa*(pi/180)
    
    
    Finv=lambda X: np.array([[A[0]+r1*cos(X[0])+r2*cos(X[1])+(r4+r5)*cos(X[3])-x_goal],
                          [D[0]+r3*cos(X[2])+r5*cos(X[3])-x_goal],
                          [A[1]+r1*sin(X[0])+r2*sin(X[1])+(r4+r5)*sin(X[3])-y_goal],
                          [D[1]+r3*sin(X[2])+r5*sin(X[3])-y_goal]])

    Jinv=lambda X: np.array([[-r1*sin(X[0]), -r2*sin(X[1]), 0, -(r4+r5)*sin(X[3])],
                            [0, 0, -r3*sin(X[2]), -r5*sin(X[3])],
                            [r1*cos(X[0]), r2*cos(X[1]), 0, (r4+r5)*cos(X[3])],
                            [0, 0, r3*cos(X[2]), r5*cos(X[3])]])
    X=Newton_Rhapson(Jinv,Finv,Xa, TOL)
    
    print(X*(180/pi))
    return X*(180/pi)
    

Xo=[[180],[90],[90],[0]]# lo que de despues de ejecutar cinamtica directa
X=np.array(Xo)
print(entradas)
angulos_inverse = inversa(int(entradas[0]),int(entradas[1]),int(entradas[2]),X)
print(angulos_inverse[0])

    
    
    

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
        global angulos_inverse    
        try:      
            
            if key.char == "u": #Motor1 adelante
                comand.angular.x = float(pasos)
                print(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint1 adelanTe')
                

            elif key.char == "j": #Motor1 atras
                comand.angular.x = -float(pasos)
                print(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint1 atras')

            elif key.char == "i": #Motor2 adelante
                comand.angular.y = float(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint2 adelante')


            elif key.char == "k": #Motor2 adelante
                comand.angular.y = -float(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint2 atras')


            elif key.char == "o": #Motor4 atras
                comand.angular.z = -float(pasos)
                print(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint3 atras')

            elif key.char == "l": #Motor4 adelante
                comand.angular.z = float(pasos)
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint3 adelante')


            elif key.char == "p": #Motor4 gripper abrir
                comand.linear.x = -float(165)
                print("letra p")
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint4 abrir gripper')


            elif key.char == ";": #Motor4 gripper cerraroom
                comand.linear.x = float(165)
                print("punto coma")
                self.publisher_comands_.publish(comand)
                self.get_logger().info('Joint4 cerrar gripper')
                

            elif key.char == "m": #Modificar el numero de pasos
                pasos = input("Cuantos grados desea moverse")
                print("Pasos fijos es " + pasos)
                
            elif key.char == "c": #Hacer cinematica inversa
                comand.angular.x = float(0)
                comand.angular.y = float(angulos_inverse[0])
                comand.angular.z = float(angulos_inverse[1])
                self.publisher_comands_.publish(comand)
                self.get_logger().info("Publicar cinematica inversa")
        
        except Exception as e:
            print(e)
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


