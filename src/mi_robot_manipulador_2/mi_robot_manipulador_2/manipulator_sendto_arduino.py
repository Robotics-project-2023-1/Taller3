import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray #Tipo de mensaje recibido en el topico robot_bot_teclas
import serial
import time
ser = serial.Serial("/dev/ttyACM0", baudrate=230400) #Modificar el puerto serie de ser necesario
ser.flush()
lista_a = [0,0,0,0,0,0,0] #
#dar permiso al puerto sudo chmod 666 /dev/ttyACM0
from time import sleep

class ManipulatorSendtoArduino(Node):

    def __init__(self):
        super().__init__('manipulator_sendto_arduino')
        self.subscription = self.create_subscription( Int32MultiArray,'/robot_manipulator_comands', self.comandos_callback,10)
        
    def comandos_callback(self, comando):
        global lista_a
        lista_n = [comando.data[0],comando.data[1],comando.data[2],comando.data[3],comando.data[4],comando.data[5], comando.data[6]]
        #lista_n = [selector, linear, angular, serv1, serv2, serv3, serv4]
        print("Comando para arduino recibido: " + str(lista_n))
        
        if(lista_n != lista_a):
            ser.write(f"{comando.data[0]},{comando.data[1]},{comando.data[2]},{comando.data[3]},{comando.data[4]}, {comando.data[5]}, {comando.data[6]} \n".encode())
            #ser.write(f"{1},{0},{0},{0},{0},{0},{0},{0}\n".encode()) #Para detener
            time.sleep(0.1)
        lista_a = lista_n

        #if ser.inWaiting() > 0:
            #entrada = ser.readline().decode('utf-8').rstrip()
            #info_arduino = entrada.split(',')
            #print(info_arduino)
            #ser.flush()
          

def main(args=None):
    rclpy.init(args=args)
    manipulator_sendto_arduino = ManipulatorSendtoArduino()
    rclpy.spin(manipulator_sendto_arduino)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
