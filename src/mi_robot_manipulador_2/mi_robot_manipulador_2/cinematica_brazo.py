import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import sys, termios, tty, select
from geometry_msgs.msg import Twist
from math import cos
from math import sin
from math import pi
from time import time

TOL=1e-6 # error cuadrático de respuesta
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

    
    
while True:
    entradas = input("Coordenadas: ").split(",")
    Xo=[[180],[90],[90],[0]]# lo que de despues de ejecutar cinamtica directa
    X=np.array(Xo)
    print(entradas)
    angulos_inverse = inversa(int(entradas[0]),int(entradas[1]),int(entradas[2]),X)
    print(angulos_inverse)
    
