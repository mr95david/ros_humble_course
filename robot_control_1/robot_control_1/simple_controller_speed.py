#!/usr/bin/env python3
# El siguiente codigo se realiza para crear el controlador de velocidad del robot simulado del curso de udemy,
# Es necesario realziar las correspondientes modificaciones para el robot solver.
# Seccion de importe de librerias
import rclpy
from rclpy.node import Node
from rclpy.time import Time
# Constante para cambio de nanoseconds to seconds
from rclpy.constants import S_TO_NS
# importe de librerias de interfaz
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
# Librerias utilitarias
import numpy as np

# Seccion de nodo
class ControllerVelocity(Node):
    def __init__(self) -> None:
        super().__init__("simple_controller")

        # Este parametro se anexa para que el usuaio pueda agregar el tamanho de ruedas del robot especifico,
        self.declare_parameter("wheel_radius", 0.033)
        # Tambien se agrega un parametro para la designacion del la longitud de separacion de las ruedas
        self.declare_parameter("wheel_separation", 0.17)

        # Se crean instancias que almacenan la posicion anterior de las ruedas
        # Esto con el fin de calcular la posicion actual del robot de acuerdo a la velocidad lineal y angular dada
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_vel_ = 0.0
        # Tambien la instancia de tiempo de diferencia
        self.prev_time_ = self.get_clock().now()

        # Usamos las funciones para obtener los valores de parametros creados
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        # Validacion de lectura de parametros
        self.get_logger().info(f"Se usa el parametro de radio de ruedas: {self.wheel_radius}")
        self.get_logger().info(f"Se usa el parametro de separacion de ruedas: {self.wheel_separation}")

        # Se crea el publicador de velocidad de las ruedas tambien
        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray,
            "simple_velocity_controller/commands",
            10
        )
        
        # Creacion de subscriptor de velocidad
        self.vel_sub_ = self.create_subscription(
            TwistStamped,
            "robotController/cmd_vel",
            self.velocity_callback,
            10
        )
        # De esta manera se tendria que hacer un subscriptor que obtenga el valor de posicion de los encoders
        self.joint_sub_ = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_callback,
            10
        )

        # Seccion de variables utilizadas 
        self.speed_conversion_ = np.array([
            [self.wheel_radius/2, self.wheel_radius/2],
            [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]
        ])

        # Validacion de valores obtenidos
        self.get_logger().info(f"El valor obtenido de la matriz de conversion es {self.speed_conversion_}")

    # Funcion de callback de velocidad 
    def velocity_callback(self, msg: TwistStamped):
        robot_speed = np.array([
            [msg.twist.linear.x], # Asi extraemos la componente de velocidad linear 
            [msg.twist.angular.z] # De aca extraemos la componente de velocidad angular
        ])

        wheel_speed = np.matmul(np.linalg.inv(
            self.speed_conversion_), robot_speed
        ) # Esta seccion realiza la multiplicacion entre las matrices de los valores de velocidad lineal y angular, y la matriz de rotacion calculada

        # Ahora se crea la variable que acumula la multiplicacion
        wheel_speed_msg = Float64MultiArray()

        # Asignacion de valores en la variable pa publicar
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        # Seccion de publicacion de valor resultante
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    # Creamos el callback que recibira los valores de las articulaciones
    def jojoint_callback(self, msg: JointState):
        # Este corresponde al delta de posicion de la rueda izquierda
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        # De la misma manera la posicion de la rrueda derecha
        dp_right = msg.position[0] - self.right_wheel_prev_vel_

        # Tambien se crea la variable que almacena el valor del delta de tiempo
        # la funcion time convierte los valores en un valor de tiempo
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # Se asignan los valores nuevos a las variables de memoria
        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_vel_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Se calcula el valor de fi (Posicion)
        fi_left = dp_left / (dt/S_TO_NS)
        fi_right = dp_right / (dt/S_TO_NS)

        # Ahora se debe calcular los valores de velocidad desde esta perspectiva de los encoders
        linear = (self.wheel_radius*fi_right + self.wheel_radius*fi_left) / 2
        # Ahora se hace el calculo para el valor de velocidad angular
        angular = (self.wheel_radius*fi_right + self.wheel_radius*fi_left) / self.wheel_separation

        # 

def main(args = None):
    # Inicializacion de rclpy
    rclpy.init(args = args)
    # Nodo creado
    simple_controller = ControllerVelocity()
    
    # Ejecucion de nodo 
    rclpy.spin(simple_controller)

    # Fin de la ejecucion
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
