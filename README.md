# Taller3

## Definicion del volumen de trabajo del brazo

El volumen de trabajo se definio usando las siguientes ecuaciones

## Para probar inverse kinematics

Correr el nodo planner con: `ros2 run mi_robot_manipulator_2 robot_manipulator_planner`
Abrir una terminal nueva y correr: `ros2 topic pub /robot_manipulator_goal geometry_msgs/Point "{x: 1.0, y: 2.0, z: 5.0}"`

## Nodos:
Control por teclas. Tiene como input el numero de pasos que se le quieren mandar a los motores. Tiene incluida la funcion de directa() 
`ros2 run mi_robot_manipulator_2 robot_manipulator_teleop` 

Realizar cinematica inversa. Se activa cuando se le publica algo al topico /robot_manipulator_goal. Solo realiza el comando una vez, y vuelve a mandarlo si la posicion 
deseada es diferente a la anterior. Publica al nodo del arduino para mover los motores el numero de pasos indicados en la funcion inversa(). Tambien usa la funcion directa() que recibe los 
pasos dados para poder publicar a la interfaz la posicion final usando el topico /robot_manipulator_position. 
`ros2 run mi_robot_manipulator_2 robot_manipulator_planner`

Se suscribe al topico /robot_manipulator_position 
`ros2 run mi_robot_manipulator_2 robot_manipulator_interface`

Para mandar cosas al arduino
`ros2 run mi_robot_manipulator_2 manipulator_sendto_arduino`

## Instalaciones requeridas para el paquete mi_robot_manipulator_2

#### Instalar pyautogui: 
```
sudo apt update
python3 -m pip install pyautogui
sudo apt-get install scrot
sudo apt-get install python3-tk
sudo apt-get install python3-dev
```
### Instalar serial

`sudo pip3 install pyserial`

### Para habilitar screenshots:

1. `sudo nano /etc/gdm3/custom.conf`
2. WaylandEnable=false
3. guardar los cambios
4. `sudo systemctl restart gdm3`

##
