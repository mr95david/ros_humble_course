#!/usr/bin/env python3
# Seccion de importe de librerias
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
# Importamos las librerias de interfaces necesarias
# En este caso la interface de principio que establece el valor un parametro
from rcl_interfaces.msg import SetParametersResult

# Este programa, crea una serie de parametros asociados a un nodo especifico
# de tal manera que se almacenan valores necesarios para uso en otros nodos por emdio
# de un nodo.
# Descripcion de chat: archivo que se utiliza para almacenar valores de parámetros que pueden ser utilizados por nodos y componentes dentro de un sistema ROS 2. Estos archivos proporcionan una forma conveniente de definir y organizar los parámetros que pueden afectar el comportamiento de los nodos, como configuraciones de hardware, umbrales de detección, velocidades de movimiento, etc.
# Seccion de creacion de nodo
class NodoParametros(Node):
    def __init__(self) -> None:
        # Inicializacion de nodo
        super().__init__("nodo_de_parametros")

        # Creamos el primer parametro del nodo o la secciond e parametros
        # Primer: Nombre del parametro, segundo: Valor del parametro
        self.declare_parameter("simple_int_parameter", 28)
        self.declare_parameter("simple_string_parameter", "Elio")

        # Ahora creamos una funcion que va hacer uso de esos parametros
        self.add_on_set_parameters_callback(self.paramChangeCallback)

    # La siguiente funcion, ejecutara un proceso con los parametros establecidos
    def paramChangeCallback(self, params):
        # Uso de interface, para asignar un valor de un parametro
        result = SetParametersResult()

        # ciclo de ejecucion de parametros
        # Principalmente la funcion varia los valores almacenados en los parametros del nodo
        for param in params:
            # Validacion de nombre de parametro
            if param.name == "simple_int_param" and param.type == Parameter.Type.INTEGER:
                self.get_logger().info(f"Param_simple_int_param change : {param.value}")
                result.successful = True
            
            if param.name == "simple_string_parameter" and param.type == Parameter.Type.STRING:
                self.get_logger().info(f"simple_string_parameter change: {param.value}")
                result.successful = True
# Funcion main de ejecucion completa del programa creado
def main(args = None):
    # Inicializacion de rclpy
    rclpy.init(args = args)
    # Nodo creado
    nodo_parametros = NodoParametros()
    
    # Ejecucion de nodo 
    rclpy.spin(nodo_parametros)

    # Fin de la ejecucion
    nodo_parametros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # print(
    #     "Nodo ejecutandose correctamente"
    # )
    main()