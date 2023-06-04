import tkinter as tk
from tkinter import PhotoImage
from tkinter import Canvas
from tkinter import filedialog
import pyautogui
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist   #Tipo de mensaje que se publica el topico robot_position
from std_msgs.msg import String #Tipo de mensaje recibido en el topico robot_bot_teclas
from threading import Thread  #Crear threads para correr dos cosas simultaneamente
from time import sleep


posiciones = [0,0,0]
keys_pressed = []
tecla_presionada = '.'


#Thread
def create_interface():
    print("Creando interfaz")
        # Crear la ventana principal
    ventana = tk.Tk()

    # Personalizar la ventana
    ventana.title("robot_manupulator_interface")
    ventana.geometry("950x600")
    ventana.configure(bg='#ff8000')
    #Botón



    #Text_Frame
    text_frame = tk.Text(ventana, width = 35, height = 1, font=("Courier",16))
    text_frame.place(x = 470, y = 50)

    #Label para text frame
    label = tk.Label(ventana, text="Selecciona nombre para el pantallazo:", font=("Courier",15), bg = "#ff8000")
    label.place(x = 15, y = 50)


    '''
    LABELS DE IMAGENES
    '''

    #Poner label de plano de imagen 1
    label_planoxy = tk.Label(ventana, text="Plano XY", font=("Courier",15), bg = "#ff8000")
    label_planoxy.place(x = 170, y = 120)

    #Label para marcar eje x
    label_x = tk.Label(ventana, text="X", font=("Courier",15), bg = "#ff8000")
    label_x.place(x = 210, y = 440)

    #Label para marcar eje y en plano 1
    label_y1 = tk.Label(ventana, text="Y", font=("Courier",15), bg = "#ff8000")
    label_y1.place(x = 60, y = 300)


    #Poner label de plano de imagen 2
    label_planoyz = tk.Label(ventana, text="Plano YZ", font=("Courier",15), bg = "#ff8000")
    label_planoyz.place(x = 670, y = 120)

    #Label para marcar eje y en plano 2
    label_y2 = tk.Label(ventana, text="Y", font=("Courier",15), bg = "#ff8000")
    label_y2.place(x = 720, y = 440)

    #Label para marcar eje z
    label_y1 = tk.Label(ventana, text="Z", font=("Courier",15), bg = "#ff8000")
    label_y1.place(x = 560, y = 300)


    """
    CREACIÓN DE LOS CANVAS
    """
    #Posicionar imagen 1 - posicion x = 80, y = 150
    imagen = PhotoImage(file="/home/yaisa/Taller3/src/mi_robot_manipulador_2/mi_robot_manipulador_2/grid.png").subsample(1)

    ancho = imagen.width()
    alto = imagen.height()

    #Canvas1
    canvas1 = Canvas(ventana, width = ancho, height = alto)

    ancho_canvas = canvas1.winfo_width()
    alto_canvas = canvas1.winfo_height()
    canvas1.place(x = 80, y = 150)

    x = (ancho_canvas) // 2
    y = (alto_canvas) // 2

    imagen_canvas1 = canvas1.create_image(x, y, anchor=tk.NW, image=imagen)


    #Canvas2
    canvas2 = Canvas(ventana, width = ancho, height = alto)
    canvas2.place(x = 580, y = 150)
    imagen_canvas2 = canvas2.create_image(x, y, anchor=tk.NW, image=imagen)

    """
    FUNCIÓN PARA DIBUJAR DENTRO DE CADA CANVAS - (MANUALMENTE)
    
    pos_x1 = ancho//2
    pos_y1 = alto//2
    pos_y2 = ancho//2
    pos_z2 = alto//2


    def move_and_draw(event):
        global pos_x1, pos_y1, pos_y2, pos_z2
        if event.keysym == "a":
            nueva_pos_x1 = pos_x1 - 10
            canvas1.create_line(pos_x1, pos_y1, nueva_pos_x1, pos_y1, fill="green",width=5)
            pos_x1 = nueva_pos_x1
        elif event.keysym == "d":
            nueva_pos_x1 = pos_x1 + 10
            canvas1.create_line(pos_x1, pos_y1, nueva_pos_x1, pos_y1, fill="green",width=5)
            pos_x1 = nueva_pos_x1
        elif event.keysym == "w":
            nueva_pos_y1 = pos_y1 - 10
            nueva_pos_y2 = pos_y2 - 10
            canvas1.create_line(pos_x1, pos_y1, pos_x1, nueva_pos_y1, fill="green",width=5)
            canvas2.create_line(pos_y2, pos_z2, nueva_pos_y2, pos_z2, fill="green",width=5)
            pos_y1 = nueva_pos_y1
            pos_y2 = nueva_pos_y2
        elif event.keysym == "s":
            nueva_pos_y1 = pos_y1 + 10
            nueva_pos_y2 = pos_y2 + 10
            canvas1.create_line(pos_x1, pos_y1, pos_x1, nueva_pos_y1, fill="green",width=5)
            canvas2.create_line(pos_y2, pos_z2, nueva_pos_y2, pos_z2, fill="green",width=5)
            pos_y1 = nueva_pos_y1
            pos_y2 = nueva_pos_y2
        elif event.keysym == "k":
            nueva_pos_z2 = pos_z2 + 10
            canvas2.create_line(pos_y2, pos_z2, pos_y2, nueva_pos_z2, fill="green",width=5)
            pos_z2 = nueva_pos_z2
        elif event.keysym == "l":
            nueva_pos_z2 = pos_z2 - 10
            canvas2.create_line(pos_y2, pos_z2, pos_y2, nueva_pos_z2, fill="green",width=5)
            pos_z2 = nueva_pos_z2




    ventana.bind("<KeyPress>", move_and_draw)
"""

    """
    PARA TOMAR UN PANTALLAZO
    """

    def guardar_pantallazo_xy():
        # Obtener el nombre de archivo ingresado por el usuario
        nombre_archivo = text_frame.get("1.0", tk.END).strip() + "_xy"

        # Mostrar el cuadro de diálogo para seleccionar la ubicación de guardado
        ruta_guardado = filedialog.asksaveasfilename(defaultextension=".png", initialfile=nombre_archivo)

        # Verificar si se seleccionó una ubicación de guardado
        if ruta_guardado:
            # Obtener las coordenadas y dimensiones del lienzo
            x = canvas1.winfo_rootx() - ventana.winfo_rootx()
            y = canvas1.winfo_rooty() - ventana.winfo_rooty()
            width = canvas1.winfo_width()
            height = canvas1.winfo_height()

            # Capturar la imagen del lienzo utilizando pyautogui
            imagen = pyautogui.screenshot(region=(x+75, y+65, width, height))

            # Guardar la imagen en el archivo seleccionado
            imagen.save(ruta_guardado)
            print("Pantallazo guardado como", ruta_guardado)


    def guardar_pantallazo_yz():
        # Obtener el nombre de archivo ingresado por el usuario
        nombre_archivo = text_frame.get("1.0", tk.END).strip() + "_yz"

        # Mostrar el cuadro de diálogo para seleccionar la ubicación de guardado
        ruta_guardado = filedialog.asksaveasfilename(defaultextension=".png", initialfile=nombre_archivo)

        # Verificar si se seleccionó una ubicación de guardado
        if ruta_guardado:
            # Obtener las coordenadas y dimensiones del lienzo
            x = canvas2.winfo_rootx() - ventana.winfo_rootx()
            y = canvas2.winfo_rooty() - ventana.winfo_rooty()
            width = canvas2.winfo_width()
            height = canvas2.winfo_height()
            # Capturar la imagen del lienzo utilizando pyautogui
            imagen = pyautogui.screenshot(region=(x+75, y+65, width, height))

            # Guardar la imagen en el archivo seleccionado
            imagen.save(ruta_guardado)
            print("Pantallazo guardado como", ruta_guardado)

    def guardar_pantallazo():
        guardar_pantallazo_xy()
        guardar_pantallazo_yz()



    boton = tk.Button(ventana, command = guardar_pantallazo, text = 'Guardar pantallazo', width = 17, height = 3, font=("Courier",15))
    boton.place(x = 40, y = 500)



    #Para coordinar movimientos
    while True:
        print("Dibujando")
        print(posiciones)
        xd = (posiciones[0]+2.27)*88.3+200
        yd = ((posiciones[1]+2.27)*88.3-402.5)*(-1)+95
        zd = (posiciones[2] + 2.27) * 100 + 200

        sleep(0.2)
        canvas1.create_line(xd,yd,xd+2,yd+2,fill='green',width=5)
        canvas2.create_line(yd,zd,yd+2,zd+2,fill='blue',width=5)

        ventana.update_idletasks()
        ventana.update()

    """EJECUCIÓN DE LA INTERFAZ"""
    # Ejecutar el bucle principal de la aplicación
    #ventana.mainloop()


class RobotManipulatorInterface(Node):
    def __init__(self):
        super().__init__("robot_manipulator_interface")
        self.subscription = self.create_subscription(Twist, 'robot_manipulator_position',self.listener_callback_posicion,10)
        self.subscription
        self.subscription = self.create_subscription(String, 'robot_manipulator_teclas',self.listener_callback_teclas,10)
        self.subscription
        print("cliente creado")

    def listener_callback_posicion(self,msg):
        posiciones[0] = msg.linear.x
        posiciones[1] = msg.linear.y
        posiciones[2] = msg.linear.z

    def listener_callback_teclas(self,msg):
        tecla_presionada = msg.data
        keys_pressed.append(tecla_presionada)



def main(args=None):

    rclpy.init(args=args)
    t = Thread(target=create_interface)
    t.start()
    robot_manipulator_interface = RobotManipulatorInterface()
    rclpy.spin(robot_manipulator_interface)
    t.join()
    robot_manipulator_interface.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()


