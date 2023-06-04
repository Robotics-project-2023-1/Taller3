import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String #Tipo de mensaje recibido en el topico robot_bot_teclas
import serial
import time
ser = serial.Serial("/dev/ttyACM0", baudrate=230400) #Modificar el puerto serie de ser necesario
ser.flush()
#dar permiso al puerto sudo chmod 666 /dev/ttyACM0
from threading import Thread  #Crear threads para correr dos cosas simultaneamente
from time import sleep
global lista_a
lista_a = [0,0,0,0] #uuuujjjjiiikolluuuujjjjiiikkkkkkkool
global control 
control = False

class ManipulatorSendtoArduino(Node):

    def __init__(self):
        super().__init__('manipulator_sendto_arduino')
        self.subscription = self.create_subscription( Twist,'/robot_manipulator_comands', self.comandos_callback,10)
        

    def comandos_callback(self, comando):
        global lista_a
        global read_serial
        global x
        global y
        global control
        global posiciones
        motor1 = int(comando.angular.x)
        motor2 = int(comando.angular.y)
        motor3 = int(comando.angular.z)
        motor4 = int(comando.linear.x)
        lista_n = [motor1, motor2, motor3, motor4]
        print(lista_n)

        ser.write(f"{motor1},{motor2},{motor3},{motor4} \n".encode())
        ser.write(f"{0},{0},{0},{0}\n".encode())
        control = True
        time.sleep(0.1)
       
def main(args=None):
    rclpy.init(args=args)
    #t = Thread(target=leer_encoders) #inicia un segundo hilo en el cual va a correr la interfaz
    #t.start() #inicia la interfaz
    manipulator_sendto_arduino = ManipulatorSendtoArduino()
    rclpy.spin(manipulator_sendto_arduino)
    #t.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
