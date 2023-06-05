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
        ang_serv1 = coordenadas_goal.x
        ang_serv2 = coordenadas_goal.y
        ang_serv3 = coordenadas_goal.z
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
        x_final = 1*grados_movidos[0]
        y_final = 1*grados_movidos[1]
        z_final = 1*grados_movidos[2]
        
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


