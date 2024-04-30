#!/usr/bin/env python3
# En el siguiente programa se realiza la primer prueba de un servicion
# Considero que este medio es el medio mas util para realizar, o ejecutar tareas
# desde nodos, como las funciones de reconocimiento o comunicacion de manera constante

# Seccion de importe de librerias
import rclpy
from rclpy.node import Node
# Importar la interface creada
from ntarobot_msg.srv import AddTwoNumbers

# Creacion de clase del nodo
class SimpleServiceNode(Node):
    def __init__(self):
        # inicializa el nodo
        super().__init__("simple_service")

        # Se crea la instancia del servicio
        self.service_instace_ = self.create_service(
            AddTwoNumbers,
            "add_two_ints",
            self.service_callback
        ) # De esta manera creamos un servicio con una interface creada tambien
        # en otro paquete disponible

        # Validamos la creacion del servicio
        self.get_logger().info("Servicio para agregar 2 valores enteros ha sido creado")

    # Creamos la funcion de callback para el servicio necesario
    def service_callback(self, req, res):
        # Visualizacion la solicitud para atribuir el servicio
        self.get_logger().info(f"Nueva solicitud dada para los numeros: {req.a} y {req.b}")
        # Aca calculamos la respuesta deseada susando el servicio
        res.sum = req.a + req.b
        # Finalmente se imprime el resultado obtenido del servicio utilizado
        self.get_logger().info(f"El resultado de la suma es: {res.sum}")

        return res
    
# Al igual que cualquier nodo, creamos una funcion main, donde se creara y ejecutara el servicio
def main(args = None):
    # Lo primero es inicializar la libreria de ros
    rclpy.init(args = args)

    # Segundo la creacion del nodo de ejecucion
    simple_service_ = SimpleServiceNode()
    # Tercero se ejecuta el spin del nodod creado
    rclpy.spin(simple_service_)

    # Se debe destruir el nodo al acabar
    simple_service_.destroy_node()
    # Finalmente apagar la libreria
    rclpy.shutdown()

# Para dar ejecucion con si fuera un nodo de ros2 
if __name__ == "__main__":
    main()
