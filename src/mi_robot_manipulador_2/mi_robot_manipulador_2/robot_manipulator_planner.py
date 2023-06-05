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

msg_nuevo = Point()
msg_nuevo.x = 0.0
msg_nuevo.y = 0.0
msg_nuevo.z = 0.0
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
    
    

class RobotManipulatorPlanner(Node):
    def __init__(self):
        super().__init__('Robot_manipulator_planner')
        self.publisher_comands_ = self.create_publisher(Int32MultiArray, '/robot_manipulator_comands', 10)
        self.publisher_position_ = self.create_publisher(Point, '/robot_manipulator_position', 10)
        self.subscription = self.create_subscription(Point, '/robot_manipulator_goal',self.subscriber_callback_goal,10) #nodo se suscribe a robot_teclas
        self.subscription
        self.get_logger().info('Robot Manipulator Planner node initialized')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publisher_callback)
        self.i = 0
        
    def inversa(self, coordenadas_goal): #RODRI PON AQUI TUS ECUACIONES
        #BLA BLA PROCESO INVERSA
        
        # Example usage
        x = coordenadas_goal.x  # Desired end effector x-coordinate
        y = coordenadas_goal.y  # Desired end effector y-coordinate
        z = coordenadas_goal.z  # Desired end effector z-coordinate
        link1_length = 9  # Length of link 1
        link2_length = 9  # Length of link 2

            
        # Calculate the distance from the base to the end effector projection in the XY plane
        distance_xy = np.sqrt(x**2 + y**2)

        # Calculate the distance from the base to the end effector
        distance = np.sqrt(distance_xy**2 + z**2)

        # Check if the desired position is reachable
        if distance > link1_length + link2_length:
            print("Desired position is out of reach")


        # Calculate the angles using trigonometry
        theta2 = np.arccos((x**2 + y**2 + z**2 - link1_length**2 - link2_length**2) / (2 * link1_length * link2_length))
        theta1 = np.arctan2(y, x) - np.arctan2(link2_length * np.sin(theta2), link1_length + link2_length * np.cos(theta2))

        # Calculate the angle for the rotating base
        theta_base = np.arctan2(y, x)

        # Convert angles to degrees
        theta_base = np.degrees(theta_base)
        theta1 = np.degrees(theta1)
        theta2 = np.degrees(theta2)
        
        ang_serv1 = theta1
        ang_serv2 = theta2
        ang_serv3 = theta_base
        #BLA BLA FORMULAS
        print("Movimientos de cada servo fueron planeados. Ejecutando...")
        comando = Int32MultiArray()  #
        self.arreglo = [2,0,0,int(ang_serv1),int(ang_serv2),int(ang_serv3),-165] #Selector, lineal, angular, serv1, serv2, serv3, serv4
        print("Se mandara el comando " + str(self.arreglo))
        comando.data = list(self.arreglo)
        self.publisher_comands_.publish(comando)
        self.get_logger().info('Comando enviado a nodo sendto_arduino')
        ## Implementar alguna variable de control para cerrar el gripper
        cerrar_gripper = False
        if cerrar_gripper == True:
            self.arreglo = [2,0,0,ang_serv1,ang_serv2,ang_serv3,float(165)] #Selector, lineal, angular, serv1, serv2, serv3, serv4
            comando.data = list(self.arreglo)
            self.publisher_comands_.publish(comando)
            self.get_logger().info('Cerrar gripper fue enviado a nodo sentto_arduino')

    def directa(self, grados_movidos):  #RODRI PON AQUI TUS ECUACIONES
        #grados_movidos es [serv1, serv2, serv3] son los cambios en cada comando
        #BLA BLA PROCESO DIRECTA (EN ESTE CASO SE HACE DIRECTA SOLO PARA QUE SE VEA EN LA INTERFAZ)
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
        
        coordenadas = Point() #
        coordenadas.x = float(x_final)
        coordenadas.y = float(y_final)
        coordenadas.z = float(z_final)
        self.publisher_position_.publish(coordenadas)
        self.get_logger().info('Coordenadas publicadas')


    def publisher_callback(self):
        pass

    def subscriber_callback_goal(self,msg):
        global msg_nuevo
        if msg_nuevo != msg:
            print("Coordenadas recibidas, inicia planeacion ")
            self.inversa(msg)
            msg_nuevo = msg


def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_planner = RobotManipulatorPlanner()
    rclpy.spin(robot_manipulator_planner)
    robot_manipulator_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()