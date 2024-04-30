#!/usr/bin/env python
# Este codigo fue tomado del repositorio https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble-devel/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py
# Con el fin de acondicionarlo segun las necesidades de movimiento del robot con el que se esta aprendiendo el sistema de control
# de esta manera es importante aclarar que las modificaciones realizadas estan pensadas unicamente en un robot especifico

# Seccion de importe de librerias utilitarias
import os
import select
import sys
# Seccion de importe de librerias relacionadas a ros
import rclpy
# Seccion de importe de librerias de interfaces y medios de manejo de datos
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile
# Validacion del sistema:: En este caso se establece el tipo de software con el que se establece la comunicacion
# en teoria, si se trabaja unicamente con linux, humble ros2. solo se activara la opcion else (Posiblemente es mejor eliminar esta validacion)
if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# Seccion de definicion de variables de velocidad maxima
# variables propias
ROBOT_MAX_LIN_VEL = 0.5
ROBOT_MAX_ANG_VEL = 0.4
# BURGER_MAX_LIN_VEL = 0.22
# BURGER_MAX_ANG_VEL = 2.84

# WAFFLE_MAX_LIN_VEL = 0.26
# WAFFLE_MAX_ANG_VEL = 1.82
# Variables que definen el valor de cambio de velocidad
LIN_VEL_STEP_SIZE = 0.025
ANG_VEL_STEP_SIZE = 0.025

# Busqueda de validacion del robot _ tipo de modelo que se esta utilizando
# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL'] # TODO: Esta validacion no es necesaria dado que no se esta trabajando con ningun modelo de Turtlebot
# Simplemente ingresa en las variables de entorno y extrae el valor relacionado con la variable de interes, la salida es str
MODEL_NAME = 'Robot_defecto'

# Instrucciones de uso
msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""
# variable de estado de comunicacion
e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -ROBOT_MAX_LIN_VEL, ROBOT_MAX_LIN_VEL)
    # if TURTLEBOT3_MODEL == 'burger':
    #     return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    # else:
    #     return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -ROBOT_MAX_ANG_VEL, ROBOT_MAX_ANG_VEL)
    # if TURTLEBOT3_MODEL == 'burger':
    #     return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    # else:
    #     return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    #pub = node.create_publisher(TwistStamped, '/robotController/cmd_vel', qos)
    #pub = node.create_publisher(TwistStamped, '/robot_new_controller/cmd_vel', qos)
    pub = node.create_publisher(TwistStamped, '/ntaRobot/cmd_vel', qos)
    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = TwistStamped()

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.twist.linear.x = control_linear_velocity
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = control_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = TwistStamped()
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0

        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()