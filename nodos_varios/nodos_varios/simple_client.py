#!/usr/bin/env python3
# Ahora el siguiente codigo, corresponde a la segunda parte, que se refiere a 
# el cliente que va a interactuar con el servidor creadoanteriormente
# Primero creamos la seccion de importe de librerias
import rclpy
from rclpy.node import Node
# Es necesario importar las interfaces creadas del servidor
from ntarobot_msg.srv import AddTwoNumbers
# Se agrega una libreria para manipulacion del sistema
import sys

# Ahora creamos la seccion  de la clase donde se ejecutara las funciones del nodo
class ClientNode_owm(Node):
    def __init__(self, a, b) -> None:
        # Esta seccion corresponde a inicializacion del nodo
        super().__init__("simple_client_server")
        # Primero es necesario crear la instancia del cliente
        self.client_ = self.create_client(
            AddTwoNumbers, # Agregamos la interface que es necesaria para este servicio-cliente
            "add_two_ints"
        )

        # Ahora es necesario establecer un loop, que considera la interaccion del cliente con el servidor
        while not self.client_.wait_for_service(timeout_sec = 1.0): # Se estable un tiempo de ejecucion del loop
            self.get_logger().info("Servicio disponible, esperando nueva orden ...") # Se genera un mensaje de validacion de ejecucion del nodo}

        # Ahora se establece la conexion del request
        self.req_ = AddTwoNumbers.Request()

        # Donde tambien se da el valor de entrada para cada una de las variables de la interfaz
        self.req_.a = a
        self.req_.b = b

        # Y se ejecuta de manera final la funcion apra el cliente
        self.future_re = self.client_.call_async(self.req_) # Para esto se usa un medio de comunicacion asincrona
        # Se agrega un valor que valida la ejecucion
        self.future_re.add_done_callback(self.response_callback) # En este se debe agregar la funcion que funciona como callback

    def response_callback(self, future):
        # Valida el uso del call back
        self.get_logger().info(f"La respuesta del servicio es: {future.result().sum}")
        
# Como todos lo nodos creados, este necesita una funcion main que ejecutara el nodo
def main(args = None):
    # Lo primero es inicializar la libreria de ros
    rclpy.init(args = args)

    # En este caso se usaran funciones que interactuaran como una respuesta del sistema al nodo
    if len(sys.argv) != 3: # esta funcion valida que los argumentos ingresados correspondan a 3, uno por cada argumento de la interfaz
        print("Error! Numero de argumentos diferente al necesario para la ejecucion del servicio.")

        return -1 # Validacion de error normal

    # Luego se define la inicialziacion del objeto correspondiente a la funcion del nodo
    simple_service = ClientNode_owm(int(sys.argv[1]), int(sys.argv[2]))
    # Luego se ejecuta el nodo
    rclpy.spin(simple_service)

    # Se apaga el nodo en cuanto se haya utilizado
    rclpy.shutdown()


# Para dar ejecucion con si fuera un nodo de ros2 
if __name__ == "__main__":
    main()