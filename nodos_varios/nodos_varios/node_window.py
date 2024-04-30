#!/usr/bin/env python3
# Seccion de importe de librerias
import rclpy
from rclpy.node import Node

# Libreria para la interfaz 
from tkinter import *
from tkinter.constants import NORMAL, DISABLED
# import math

# Importe de interfaces
from solver_untils.msg import MotorOrder

# Seccion de clase de creacion de nodo
class  GuiTestM(Node):
    def __init__(self) -> None:
        super().__init__("Nodo_control_velocidades")
        # Seccion sin funcionalidad de publicador de orden de driver
        self.publisher = self.create_publisher(MotorOrder, '/comm_motors', 10)
        # Seccion de inicializacion de subscriptores
        self._init_subs()

        # Seccion de creacion ventana
        self.tk = Tk() # Inicio de ventana
        self.tk.title("Control de velocidad de motor") # Nombre de ventana general
        root = Frame(self.tk) # Base de ventana
        root.pack(fill=BOTH, expand=True) # Distibucion por defecto de la ventana

        # Mensaje de ventana
        Label(root, text="motor Control").pack()

        mode_frame = Frame(root) # Tipo
        mode_frame.pack(fill=X) # Borde de la ventana

        # Sin funcion para la modificacion de pmw

        # Cambio de rangos, actualmente sin funcion aparente
        # Seccion para modificar los metros por segundo
        slider_max_frame = Frame(root)
        slider_max_frame.pack(fill=X)
        # Seccion de cambio de velocidad
        self.slider_max_label = Label(slider_max_frame, text="Max Rev/sec", state=DISABLED)
        # Localizacion de seccion
        self.slider_max_label.pack(side=LEFT)
        # Seccion de maximo de velocidad
        self.slider_max_val_box = Entry(slider_max_frame, state = DISABLED)
        self.slider_max_val_box.pack(side=LEFT)
        # Boton de actualizacion de opciones
        # self.max_val_update_btn = Button(slider_max_frame, text='Update', command=self.update_scale_limits, state="disabled")
        # self.max_val_update_btn.pack(side=LEFT)

        # Seccion de barra de cambio de valores pra aumento o disminucion de velocidad
        m1_frame = Frame(root)
        m1_frame.pack(fill=X)
        # Primera barra de cambio de valor, se entiende que para el motor 1
        Label(m1_frame, text="Motor 1").pack(side=LEFT)
        # Este valor se da desde 0 a 255 segun la documentacion del driver.
        self.m1 = Scale(m1_frame, from_=50, to=200, orient=HORIZONTAL)
        self.m1.set(128)
        self.m1.pack(side=LEFT, fill=X, expand=True)

        m2_frame = Frame(root)
        m2_frame.pack(fill=X)
        # Segunda barra de cambio de valor, se entiende que para el motor 2
        Label(m2_frame, text="Motor 2").pack(side=LEFT)
        self.m2 = Scale(m2_frame, from_=50, to=200, resolution=1, orient=HORIZONTAL)
        # Numero por defecto de la escala
        self.m2.set(128)
        self.m2.pack(side=LEFT, fill=X, expand=True)

        # Seccion de botones para enviar los valores necesarios
        motor_btns_frame = Frame(root)
        motor_btns_frame.pack()
        Button(motor_btns_frame, text='Send Once', command=self.send_motor_once).pack(side=LEFT)
        Button(motor_btns_frame, text='Send Cont.', command=self.show_values, state="disabled").pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Send', command=self.show_values, state="disabled").pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Mot', command=self.stop_motors).pack(side=LEFT)
    
    # SEccion de funciones de botones
        
    def show_values(self):
        print (self.m1.get(), self.m2.get())

    # La siguiente funcion cambia el valor de velocidad en los motores
    def send_motor_once(self):
        msg = MotorOrder()
        # msg.is_pwm = self.pwm_mode
        # if (self.pwm_mode):
        msg.m_value_1 = int(self.m1.get())
        msg.m_value_2 = int(self.m2.get())
        # else:
        #     msg.mot_1_req_rad_sec = float(self.m1.get()*2*math.pi)
        #     msg.mot_2_req_rad_sec = float(self.m2.get()*2*math.pi)

        self.publisher.publish(msg)

    # La siguiente funcion detiene los motores
    def stop_motors(self):
        msg = MotorOrder()
        # msg.is_pwm = self.pwm_mode
        msg.m_value_1 = 128
        msg.m_value_2 = 128

        self.publisher.publish(msg)
    
    # Seccion de funciones para interaccion de pluggins y botones 
    # Inicializacion de subscriptores
    def _init_subs(self):
        pass

    # Funcion para la actualizacion de la ventana
    def update(self):
        self.tk.update()

def main(args = None):
    # Inicializacion de rclpy
    rclpy.init(args=args)
    # Nodo creado
    motor_gui = GuiTestM()
    # Rango de cracion de ventana
    rate = motor_gui.create_rate(20)    
    # Ejecucion durante el correcto funcionamiento del nodo
    # NOTA: Es un buen sistema para ejecutar un nodo
    while rclpy.ok():
        # Ejecutar una vez la ventana
        rclpy.spin_once(motor_gui) 
        # Actualizacion
        motor_gui.update()

    # Fin de la ejecucion
    motor_gui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    print("Ejecucion correcta de nodo")