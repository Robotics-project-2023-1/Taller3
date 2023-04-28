import tkinter as tk #libreria para crear la interfaz
from tkinter import filedialog
from tkinter import messagebox
from tkinter import Canvas
from PIL import Image, ImageTk, ImageGrab
import pyautogui
from pynput import keyboard
import os # para acceder a los archivos de la carpeta
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist   #Tipo de mensaje que se publica el topico robot_position
from std_msgs.msg import String #Tipo de mensaje recibido en el topico robot_bot_teclas
from threading import Thread  #Crear threads para correr dos cosas simultaneamente
from time import sleep
from robot_interfaces.srv import Reproducir #servicio


global quiero_txt
global posiciones
posiciones = [0,0,0]
quiero_txt = False
nombre_txt = None 
guardar_movimientos = False
filename = None
folder_path = None
tecla_presionada = '.'
keys_pressed = []
#

def creo_interfaz(): #esta funcion corre como un segundo hilo
    print("Creo la interfaz")
    root = tk.Tk()
    root.geometry('850x600')
    root.title("robot_teleop")
    root.configure(bg='#40CFFF')
    name = "brazo"
    archivo = os.path.dirname(__file__)
    f = os.path.expanduser(archivo + '/' + name + '.jpeg')
    #img = Image.open(f) #modificado para encontrar el path de la foto, si no les sirve descomenten la linea de abajo y comenten esto
    img = Image.open('/home/robotica/Taller2/src/mi_robot_2/mi_robot_2/grid.jpeg')
    imagen = img.resize((400,400))
    new_image = ImageTk.PhotoImage(imagen)
    etiqueta_imagen = tk.Label(root, image=new_image)
    etiqueta_imagen.place(x=250,y=110)
    ancho = 850
    alto = 600
    pos_x = 400
    pos_y = 300

    ###### draw
        # canvas = Canvas(root)
    canvas = Canvas(root, width = ancho, height = alto)
    canvas.configure(bg='#ff3333')
    canvas.pack()

        # canvas.create_image(10,10,anchor=tk.NW,image=new_image)

        # img = Image.open("gato.png")
        # imagen = img.resize((400,400))
        # new_image = ImageTk.PhotoImage(imagen)
    canvas.create_image(pos_x,pos_y,image=new_image)

    x1 = pos_x
    y1 = pos_y
    x2 = pos_x + 5
    y2 = pos_y + 5

    text_frame_grafica = tk.Text(root, height=1, width=35,font=("Roboto", 15))
    text_frame_grafica.place(x=350,y=26)

    label_grafica = tk.Label(root, text="Introduce el nombre de la imagen:",bg='#ff3333',font=("Roboto", 15))
    label_grafica.place(x=0,y=26)

    text_frame_archivo = tk.Text(root, height=1, width=35,font=("Roboto", 15))
    text_frame_archivo.place(x=350,y=65)

    label_grafica = tk.Label(root, text="Introduce el nombre del archivo:",bg='#ff3333',font=("Roboto", 15))
    label_grafica.place(x=0,y=65)

    def open_file():  #si se desea cambiar la imagen de fondo de interfaz 
        file_path = filedialog.askopenfilename(initialdir = "/", title = "Select file", filetypes = (("PNG files", "*.png"), ("JPEG files", "*.jpg"), ("All Files", "*.*")))
        try:
            global image
            image = Image.open(file_path)
            image = image.resize((400,400))
            render = ImageTk.PhotoImage(image)
            img = tk.Label(root, image=render)
            img.image = render
            # img.place(x=100,y=150)
            canvas.create_image(pos_x,pos_y,image=render)
        except:
            messagebox.showerror("Error", "Failed to open the image.")

    open_file_button = tk.Button(root, text="Seleccionar Imagen", command=open_file, height=2, width=17, font=("Roboto", 11))
    open_file_button.place(x=10,y=520)

    def save_screenshot(): #si se desea guardar un pantallazo de la interfaz en su estado actual
        x = root.winfo_rootx()
        y = root.winfo_rooty()
        width = root.winfo_width()
        height = root.winfo_height()
        
        # # para ubuntu:
        # # Crea una imagen a partir de la ventana actual
        ####### hacer estos pasos en consola para que funcione:
        # # 1. $ sudo nano /etc/gdm3/custom.conf
        # # 2. $ WaylandEnable=false
        # # 3. guardar los cambios
        # # 4. $ sudo systemctl restart gdm3
        image = pyautogui.screenshot(region=(x+224, y+100, width-450, height-200))
        
        
        # # para Windows:
        # Crea una imagen a partir de la ventana actual
        # dx = (width-750)/2
        # image = ImageGrab.grab((x+dx+200, y+100, x+dx+600, y + 500))

        # Guarda la imagen en un archivo
        texto = str(text_frame_grafica.get("1.0",'end-1c'))
        file_path = filedialog.asksaveasfilename(defaultextension=".png", initialfile= texto + ".png", filetypes=(("PNG files", "*.png"), ("JPEG files", "*.jpg"), ("All Files", "*.*")))
        image.save(file_path)
    
    def read_txt():
        # solicitar servicio con el nombre del archivo
        global quiero_txt
        global nombre_txt
        try:
            file_path = filedialog.askopenfilename()
            nombre = os.path.basename(file_path)
            nombre_txt = file_path #string con el path para acceder al archivo deseado
            print(nombre_txt)
            quiero_txt = True 
            print("en funcion read_txt pasa a ser True")     
        except:
            quiero_txt = False
    
    def save_to_txt():#en caso de solicitar un txt con las teclas presionadas
        global guardar_movimientos
        global filename
        global folder_path
        print("nombre archivo")
        filename = text_frame_archivo.get('1.0',tk.END).strip()
        folder_path = filedialog.askdirectory()
        guardar_movimientos = True
        print(keys_pressed)
        with open(folder_path + '/' + filename + ".txt", "w") as file:
            for key in keys_pressed: #keys pressed se ha guardado desde que inicio a correr la interfaz
                file.write(key + '\n')
        pass

        # Crear botones
    save_screenshot_button = tk.Button(root, text="Tomar pantalla", command=save_screenshot, height=2, width=17, font=("Roboto", 11))
    save_screenshot_button.place(x=10,y=450)

    save_text_button = tk.Button(root, text="Guardar movimientos", command = save_to_txt, height=2, width=19, font=("Roboto", 11))
    save_text_button.place(x=630, y=520)

    read_movements_button = tk.Button(root, text="Reproducir movimientos", command = read_txt, height=2, width=19, font=("Roboto", 11))
    read_movements_button.place(x= 630, y = 450)

           
    while True:
        global posiciones
        xd = -posiciones[0]+400
        yd = posiciones[1]+400
        if xd <= 200:
            xd = xd + 400
        if xd >= 600:
            xd = xd - 400
        if yd <= 100:
            yd = yd + 400
        if yd >= 500:
            yd = yd - 400
        #print(xd, ' ', yd)
        sleep(0.2)
        canvas.create_rectangle(xd, yd, xd+2, yd+2, fill='green', outline='green')

        canvas.create_rectangle(200, 100, 200+5, 100+5, fill='green', outline='blue')
        canvas.create_rectangle(600, 500, 600+5, 500+5, fill='green', outline='blue')
        root.update_idletasks() #estos 2 comandos reemplazan el root.mainloop()
        root.update()
        

class Robot_interface(Node):
    
    def __init__(self):
        global quiero_txt
        super().__init__('robot_interface')
        self.subscription = self.create_subscription(String, 'Teclas',self.listener_callback_teclas,10) #nodo se suscribe a robot_teclas
        self.subscription
        self.subscription = self.create_subscription(Twist, 'posiciones',self.listener_callback_pos,10) 
        self.subscription 
        self.timer = self.create_timer(1.0,self.listener_callback_servicio)
        self.cli = self.create_client(Reproducir, 'reproducir')
        print("Cliente creado")
        while not self.cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info('Service not available, waiting again...')
        
        
    def send_request(self, nombre_txt):
        global quiero_txt
        if quiero_txt == True:  #Booleano que activa el servicio
            self.request = Reproducir.Request() #solicitar servicio
            self.request.nombre = nombre_txt #mandar la ubicacion del archivo como string<aawwssdddwwwwsswwaawwwwssdddaa
            self.future = self.cli.call_async(self.request)
            self.future.add_done_callback(self.handle_response)
            quiero_txt = False
    
    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info("Servicio finalizado")
        except Exception as e:
            self.get_logger().error("No fue posible llamar el servicio por: %s" % str(e))
            

    def listener_callback_teclas(self, msg): # lee las teclas que se presionan en el nodo robot_teleop
        print("Tecla")
        tecla_presionada = msg.data
        keys_pressed.append(tecla_presionada) #almacena las teclas en una lista para despues guardarlas en un txt de ser necesario

    def listener_callback_pos(self, msg): # lee las teclas que se presionan en el nodo robot_teleop
        global posiciones
        posiciones[0] = msg.linear.x
        posiciones[1] = msg.linear.y
        posiciones[2] = msg.linear.z
        print(posiciones)

    def listener_callback_servicio(self):
        global quiero_txt
        global nombre_txt
        if quiero_txt == True:
            print("voy a mandar request posicion")
            self.send_request(nombre_txt) #solicitar servicio 
            quiero_txt = False

        
        

def main(args=None):
    
    rclpy.init(args=args)
    t = Thread(target=creo_interfaz) #inicia un segundo hilo en el cual va a correr la interfaz
    t.start() #inicia la interfaz
    robot_interface = Robot_interface()#inicia el nodo
    rclpy.spin(robot_interface)
    t.join()
    robot_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()