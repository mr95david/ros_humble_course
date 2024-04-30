#!/usr/bin/env python
import rclpy
from rclpy.node import Node
# Mensajes
from std_msgs.msg import String
# Almacenamiento
import csv

class NodoTemporal(Node):
    def __init__(self):
        super().__init__('nodo_almacenamiento')

        # Variables de almacenamiento
        self.mensaje = 'Hola este es el prunto: '
        self.actual_time_ = 0 # Valor en segundos
        self.time_change_value = 0.01
        self.time_actual = self.get_clock().now().to_msg().sec

        # En este caso llevaremos un publicador
        self.public_temp_ = self.create_publisher(String, '/mensaje_prueba', 10)

        # Apertura de archivo
        self.csv_file = open('../documents/output_data.csv', 'w', newline = '')
        self.writer_ = csv.writer(self.csv_file)

        # Creacion de timer de ejecucion
        self.timer_temp_ = self.create_timer(
            self.time_change_value,
            self.callbackpublic_values
        )

    def callbackpublic_values(self):
        msg_:String = String()
        nuevo_tiempo = self.get_clock().now().to_msg().sec - self.time_actual
        self.actual_time_ += self.time_change_value
        msg_.data = self.mensaje + str(self.actual_time_)

        self.writer_.writerow([nuevo_tiempo, msg_.data])

        self.public_temp_.publish(msg_)

    def closeNode(self):
        self.csv_file.close()

# Me gusta esta manera de cerrar los nodos
def main(args = None):
    rclpy.init(args=args)

    data_logger = NodoTemporal()
    try:
        rclpy.spin(data_logger)
    except KeyboardInterrupt:
        pass
    finally:
        data_logger.closeNode()
        data_logger.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__mian__":
    main()




