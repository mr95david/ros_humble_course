#!/usr/bin/env python3

# Library Section
import rclpy
from rclpy.node import Node
# Msg
from geometry_msgs.msg import TwistStamped
from solver_untils.msg import MotorOrder
# untils libraries
import numpy as np

class pruebaNodo(Node):
    def __init__(self) -> None:
        super().__init__('nodo_cambio_valores')
        # Variables de instancia que se toman como medidas
        self.wheel_radius = 0.07
        self.wheel_sep = 0.405
        # Change value to each motor
        self.value_w_r = 10.273
        self.value_w_i = 10.242

        # Matriz de conversion
        self.speed_conversion_ = np.array([
            [self.wheel_radius/2, self.wheel_radius/2],
            [self.wheel_radius/self.wheel_sep, -self.wheel_radius/self.wheel_sep]
        ])

        # Init subscriber
        self.sub_cmd_vel_ = self.create_subscription(
            TwistStamped,
            '/robotController/cmd_vel',
            self.callbackFunction,
            10
        )
        self.sub_cmd_vel_

        # Init publisher motor
        self.publisher_motor_ = self.create_publisher(
            MotorOrder,
            '/comm_motors',
            10
        )
        

    def callbackFunction(self, msg: TwistStamped):
        msg_motors: MotorOrder = MotorOrder()
        #self.get_logger().info(f"La velociad lineal y a l: {msg.twist.linear.x} a: {msg.twist.angular.z}")
        # Obtencion de valores enviados
        velLineal: float = float(msg.twist.linear.x)
        velAngular: float = float(msg.twist.angular.z)

        # Matriz de velocidad de robot
        robot_speed = np.array([
            [velLineal], # Asi extraemos la componente de velocidad linear 
            [velAngular] # De aca extraemos la componente de velocidad angular
        ])

        wheel_speed = np.matmul(np.linalg.inv(
            self.speed_conversion_), robot_speed
        ) 

        # self.get_logger().info(f"La velociad de ruedas en rd/s d: {wheel_speed[1, 0]} i: {wheel_speed[0, 0]}")
        # self.get_logger().info(f"La velociad de ruedas en val digiatl d: {wheel_speed[1, 0]*self.value_w_r+128} i: {wheel_speed[0, 0]*self.value_w_i+128}")
        
        vel_rot_wd = wheel_speed[1, 0]*self.value_w_r+128
        vel_rot_wi = wheel_speed[0, 0]*self.value_w_i+128

        # Asignar valor de controlador
        msg_motors.m_value_1 = int(vel_rot_wd)
        msg_motors.m_value_2 = int(vel_rot_wi)

        # Publicacion de valor
        self.publisher_motor_.publish(msg_motors)


def main(args = None):
    # Inicializacion de rclpy
    rclpy.init(args = args)
    # Nodo creado
    simple_controller = pruebaNodo()
    # Cierre de nodos
    try:
        rclpy.spin(simple_controller)
    except KeyboardInterrupt:
        pass
    finally:
        simple_controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()