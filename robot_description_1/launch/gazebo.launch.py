# El siguiente paquete busca crear el launch para los archivos de gazebo referentes al robot
# de esta manera se llevara a cabo la simulacion del robot unicamente en gazebo, teniendo en 
# los valores y las fisiscas atribuidas
from launch import LaunchDescription
from launch_ros.actions import Node # Esta libreria nos permite llamar una libreria en un paquete especifico
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription # Esta libreria nos permite acceder a la declaracion de argumentos
from launch_ros.parameter_descriptions import ParameterValue # Esta libreria nos permite describir de mejor manera los archivos en el launch
from launch.substitutions import Command, LaunchConfiguration # Extrae la funcion enviar un comando en especifico
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Seccion de librerias utilitarias
import os
from os import pathsep
# Seccion de librerias utilitarias de ros2
from ament_index_python.packages import get_package_share_directory, get_package_prefix

# funcion obligatoria para crear el launch en ros2
def generate_launch_description():
    # Antes de esto debemos definir los paquetes de descripcion para la simulacion
    # Esta variable llama al paquete instalado en la carpeta share al realizar el colcon build
    ntarobot_description = get_package_share_directory('robot_description_1')
    # Define un prefijo de acceso tambien
    ntarobot_description_prefix = get_package_prefix('robot_description_1')
    # Diferencias de codigo:
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    # Esta unica diferencia, solo crea una variable que identifica la ruta del paquete de gazebo_ros

    # Usamos la misma estructura, debido a que debemos usar el modelo del robot creado en el urdf
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(
                ntarobot_description,
                "urdf", "ntarobot.urdf.xacro"
            ), # Esta funcion localiza la ruta especifica de la descripcion que estamos utilizando
        description = "Absolute path to robot URDF file"
    )

    # Definimos la ruta del modelo de la descripcion
    model_path = os.path.join(ntarobot_description, "models")
    # A esta ruta le extraemos la ruta sin el valor de share
    model_path += pathsep + os.path.join(ntarobot_description_prefix, "share")
    #print(model_path)
    # Creamos tambien la variable del entorno
    # Lo que hace es que establece la ruta especifica del modelo del robot que se va a simular, es decir
    # es como si volviera la ruta del robot un valor por defecto para gazebo
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

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

    # Ahora tambien debemos crear una variable que inicialice un servidor de gazebo
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                #get_package_share_directory("gazebo_ros"), # Esta linea llamada a la direccion donde se encuentre el paquete con el nombre especificado
                gazebo_ros_dir,
                "launch", "gzserver.launch.py" # Seguido se agregan las ejecuciones correspondientes al paquete
            )
        )
    )
    # Tambien deberemos crear una variable que cree un cliente de gazebo, para hacer uso de las funciones de gazebo en las 2
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                # get_package_share_directory("gazebo_ros"), # Paquete que se va a usar
                gazebo_ros_dir,
                "launch", "gzclient.launch.py" # Ejecuciones
            )
        )
    )

    # La ultima variable corresponde a la encargada de lanzar el modelo del robot en el entorno de gazebo
    spawn_robot = Node(
        package="gazebo_ros", # Nombre del paquete que contiene el nodo
        executable="spawn_entity.py", # Este especifica el ejecutable del nodo que se esta llamand
        arguments=["-entity", "ntarobot", "-topic", "robot_description"], # Especifica los argumentos que se ingresan con la ejecucion del nodo
        output="screen" # Finalmente el medio por el cual se mostrara el resultado
    )

    # Ejecucion de todos los nodo llamados en la funcion de generate launch
    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])

# El codigo se asemeja en 100% al codigo de referencia, realizando los cambios de funcionamiento 
# segun se establece por los nombres de los paquetes