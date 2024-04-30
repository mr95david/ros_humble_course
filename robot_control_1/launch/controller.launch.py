# Librerias
from launch import LaunchDescription
from launch_ros.actions import Node
# Se importa la funcion para usar un argumento de launch
from launch.actions import DeclareLaunchArgument, GroupAction
# Se agrega tambien la funcion para leer la configuracion especifica del launch
from launch.substitutions import LaunchConfiguration
# Incluimos tambien una funcion para establecer condiciones en la ejecucion de nodos
from launch.conditions import IfCondition, UnlessCondition

# Ejecuccion de funcion de launch
def generate_launch_description():
    # Aqui declaramos los argumentos para la ejecucion especifica de un nodo
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value = "False"
    )
    # Se declara el argumento del radio de las ruedas
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value = "0.033"
    )
    # De igual manera hay que definir el valor de la separacion entre las ruedas
    wheel_sep_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value = "0.17"
    )

    # Se crea un argumento que define cual de los controladores se va a usar
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value = "True"
    )

    # Llamado de configuracion de launch
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_sep = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller") # este argumento es para definir cual de los controladores se va a usar para el movimiento del robot

    # Nodo de ejecucion de spawner
    joint_state_broad_caster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Spawnear controlador de ruedas
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot_new_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition = UnlessCondition(use_simple_controller)
    )

    # Se crea una clase que comprende un grupo de acciones
    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions = [
            # Nodo de ejecucion de simple_controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            ),
            # Se crea la clase del nodo, donde usaremos el nodo creado para el control de velocidad
            Node(
                package = "robot_control_1", # Se tiene que llamar al paquete creado con el fin del control de velocidad
                executable = "simple_controller_speed.py", # Aqui se establece el nombre del ejecutable del nodo
                parameters = [{ # Se asignan los valor para los parametros ejecutables del nodo
                    "wheel_radius": wheel_radius,
                    "wheel_separation": wheel_sep
                }]
                # En el caso de esta prueba no se esta usando una condicicon relacionada a python, porque no tenemos algun paquete de cpp
                # condition = IfCondition(
                #     use_python # En este caso establecemos una condicion en el caso que se este usando el nodo de control por pyython
                # )
            )
        ]
    )

    return LaunchDescription([  
        use_python_arg, # Se definen los argumentos por defecto, y cambiables para la ejecucion del launch
        wheel_radius_arg, # Se definen los argumentos por defecto, y cambiables para la ejecucion del launch
        wheel_sep_arg, # Se definen los argumentos por defecto, y cambiables para la ejecucion del launch
        use_simple_controller_arg,
        joint_state_broad_caster_spawner,
        wheel_controller_spawner,
        simple_controller
        #simple_controller_node # Se agrega la ejecucion del nodo por el launch
    ])