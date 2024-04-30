# EN UN PRIMER PLANO CREAMOS EL LAUNCH PARA EL ROBOT DEL CURSO
# Paquete para hacer un pequeño archivo de launch para toda la descripcion del robot:
# Para esto primero importamos los paquetes necesarios para crear el launch
from launch import LaunchDescription # Esta paquete apoya en el launch de la descripcion del robot
from launch_ros.actions import Node # Esta libreria nos permite llamar una libreria en un paquete especifico
from launch.actions import DeclareLaunchArgument # Esta libreria nos permite acceder a la declaracion de argumentos
from launch_ros.parameter_descriptions import ParameterValue # Esta libreria nos permite describir de mejor manera los archivos en el launch
from launch.substitutions import Command, LaunchConfiguration # Extrae la funcion enviar un comando en especifico
# Seccion de librerias utilitarias
import os
# Seccion de librerias utilitarias de ros2
from ament_index_python.packages import get_package_share_directory

# Creamos la funcion general que permite inicializar los paquetes del robot
def generate_launch_description():
    # En esta seccion creamos los argumentos que podemos ingresar en los nodos que estamos creando
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(
                get_package_share_directory("robot_description_1"),
                "urdf", "ntarobot.urdf.xacro"
            ), # Esta funcion localiza la ruta especifica de la descripcion que estamos utilizando
        description = "Absolute path to robot URDF file"
    )

    # Seccion donde usaremos un valor del parametro por defecto
    robot_description = ParameterValue(
            Command([
                "xacro ", # Envia un comando del tipo "xacro ...comando"
                LaunchConfiguration("model") # Especifica la configuracion
            ]), value_type = str # y finalmente determina el tipo de valor que va a enviar en la configuracion
        )

    # Craemos un nodo, que almacena la funcion robot_state_publisher
    robot_state_publisher = Node(
        package = "robot_state_publisher", # Corresponde al paquete previamente instalado, del cual llamaremos el nodo
        executable = "robot_state_publisher", # Corresponde al nodo de ejecucion
        parameters = [{
                "robot_description": robot_description
            }] # Funcion y cambio de valor de los parametros
    )

    # El siguiente corresponde a un nodo para ejecutar la funcion de joint_state_publisher_gui
    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui", # Este corresponde al paquete que se esta llamando en esta seccion
        executable = "joint_state_publisher_gui" # Este es el ejecutable del nodo, dadoq ue no tiene un valor de parametro solo se lllama al ejecutable
    )

    rviz_node = Node(
        package = "rviz2", # Se llama al paquete de rviz2
        executable = "rviz2", # Se llama al ejecutable de rviz2
        name = "rviz2", # En este caso se agrega el nombre del paquete y la funcion que se esta llamando
        output = "screen", # Se define donde es el especifico de la salida
        arguments = [
                "-d",
                os.path.join(
                    get_package_share_directory("robot_description_1"), # Se especifica el paquete madre donde se encuentra el archivo de la descripcion
                    "rviz", "display.rviz" # Se establece la relacion de los directorios
                )
            ] # En esta seccion identificaremos el documento especifico donde se realizara la descripcion de las caracteristicas de rviz para la visualizacion del modelo del robot
        # En este caso es el arcvhio en la carpeta "rviz"
    )

    # Este retorna un LaunchDescription object que tiene todos los datos de la descripcion que se va a lanzar
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        #rviz_node
    ])