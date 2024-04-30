#!/usr/bin/env python3
# Seccion de import de librerias
import rclpy
from rclpy.node import Node
# Especialmente para esta actividad haremos uso de la libreria transform
import rclpy.time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster #TODO: Aprender un poco mas sobre esta libreria
# Libreria para la tranformacion dinamica
from tf2_ros import TransformBroadcaster, TransformException # Necesaria para la tranformacion dinamica entre 2 frames de refencia
from tf2_ros.buffer import Buffer # Buffer para recepcion y envio de datos
from tf2_ros.transform_listener import TransformListener
# Importe de librerias para transformaciones de valores
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse
# Importe de interfaces y mensajes necesarios
from geometry_msgs.msg import TransformStamped
from ntarobot_msg.srv import GetTransfrom

# Clase de nodo para la ejecucion de la funcion deseada
class nodo_transformation(Node):
    def __init__(self) -> None:
        super().__init__("nodo_transformacion") # El nombre que se coloque aca dirigira algunos de los valores de descripcion del nodo
        # Lo primero en este caso, sera crear un objeto relacionado a la transformacion estatica de un broad
        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self) # 
        # Instancia para realiza las transformacion dinamica de los frames
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)

        # Luego se define la transformacion estatica
        self.static_tf_stamped_ = TransformStamped() # Esto nos generara la informacion entre 2 frames que esten conectados
        # Tambien se define la variable de transformacion dinamica sellada
        self.dynamic_tf_stamped_ = TransformStamped()

        # Creamos un valor de incremento de posiciopn de pprueba, para verificar el funcionamiento de las transformaciones
        self.x_increment_ = 0.05
        self.last_x_ = 0.0

        # Modifiacion de parte 89 - curso. Transformacion de euler a quaternion, revisar anotaciones
        self.rotations_counter_ = 0 # Corresponde a la medida que cuenta las rotaciones ejecutadas
        self.last_orientation_ = quaternion_from_euler(0, 0, 0) # Esta variable corresponde a la instancia para cambiar los valores obtenidos desde los quaterniones a euler

        # Ahora agregamos el valor de incremento
        self.orientation_increment_ = quaternion_from_euler(0, 0, 0.05) # Esta corresponde al cambio de posicion en el espacio z = 0.05

        # Crearemos 2 variables de instancia mas para procesar los datos de transformacion
        # Primero creamos el buffer
        self.buffer_ = Buffer()
        self.transform_listener_ = TransformListener(self.buffer_, self)
        
        # Se definen luego algunos de los parametros necesarios para el constructor de la variable de transformacion
        self.static_tf_stamped_.header.stamp = self.get_clock().now().to_msg() # Se inicializa el tiempo de interaccion
        self.static_tf_stamped_.header.frame_id = "nta_base" # Se crea un identificador relacionado al robo, para el frame de referencia
        self.static_tf_stamped_.child_frame_id = "nta_top" # Se crea un identificador relacionado al robot, para el frame relacionado
        # Luego se declaran los valores de la matriz de transformacion para el constructor creado, esto para el plano del x, y, z
        self.static_tf_stamped_.transform.translation.x = 0.0 # Valor en el punto x
        self.static_tf_stamped_.transform.translation.y = 0.0 # Valor en el punto y
        self.static_tf_stamped_.transform.translation.z = 0.3 # Valor en el punto z
        # Luego se establece la transformacion para los cuaterniones de la matriz de rotacion
        self.static_tf_stamped_.transform.rotation.x = 0.0
        self.static_tf_stamped_.transform.rotation.y = 0.0
        self.static_tf_stamped_.transform.rotation.y = 0.0
        self.static_tf_stamped_.transform.rotation.w = 1.0 # Este valor corresponde a un valor de prueba

        # Luego se definen los parametros del broadcaster
        self.static_tf_broadcaster_.sendTransform(self.static_tf_stamped_) # empezando por el envio de la matriz de transformacion creada


        # Se valida la publicacion del matriz de transformacion creada
        self.get_logger().info(f"Publicando la matriz de transformacion entre 2 frams; frame 1: {self.static_tf_stamped_.header.frame_id} y frame 2: {self.static_tf_stamped_.child_frame_id}")

        # En este caso como se esta generando la transformacion dinamica entre 2 frames enviando y recibiendo datos
        # es necesario, usar un timer para definir los timepos de manejos de datos
        self.timer_ = self.create_timer(
            0.1, # Corresponde al tiempo de duracion de ciclo del timer
            self.timerCallback
        )

        # Siguiendo las enseñanzas del curso https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/learn/lecture/39580632#overview
        # En esta seccion continuaremos con la publicacion y escucha de los datos calculados en la matriz de transformacion del robot
        self.get_transform_srv_ = self.create_service(
            # Este metodo lo crearemos en el paquete de interfaces que usaremos
            GetTransfrom, # Hacemos uso de la funcion de gettransform, para obtener los valores publicados como se realiza en el curso
            "get_transform", # Este corresponderia al nombre o identificador del servicio
            self.getTransformCallback # Y este corresponderia a la funcion de callback de los datos

        ) # Usamos esta instancia para crear un servicio que publicara el cambio de la matriz de transformacion
    
    def timerCallback(self):
        # Todas estas transformaciones, se deben realizar en el timer, de tal manera que se actualicen los datos
        # Se inicializan los parametros de la transformacion stamped dinamica
        self.dynamic_tf_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_tf_stamped_.header.frame_id = "odom"
        self.dynamic_tf_stamped_.child_frame_id = "nta_base"
        # Ahora tambien se debe crear la matrix de transformacion dinamica
        self.dynamic_tf_stamped_.transform.translation.x = self.last_x_ + self.x_increment_ # Esto calcularia el desplazamiento de un frame sobre el eje x
        self.dynamic_tf_stamped_.transform.translation.y = 0.0
        self.dynamic_tf_stamped_.transform.translation.z = 0.0

        # Ahora agregamos el valor de cambio desde las variables de instancia creadas para el procesamiento de quaterniones
        # Esto incluye, la multiplicaicon continua del valor de incremento de 0.05 en la rotacion de quaterniones
        q = quaternion_multiply(self.last_orientation_, self.orientation_increment_) # Contendra el nuevo quaternion de posicion

        # Tambien se tiene que realizar el calculo para la seccion de rotacion (Seccion de transformacion de posicion simple)
        # self.dynamic_tf_stamped_.transform.rotation.x = 0.0
        # self.dynamic_tf_stamped_.transform.rotation.y = 0.0
        # self.dynamic_tf_stamped_.transform.rotation.y = 0.0
        # self.dynamic_tf_stamped_.transform.rotation.w = 1.0

        # Calculo de traslado de posicion desde el quaternion calcular (Seccion 89 del curso)
        self.dynamic_tf_stamped_.transform.rotation.x = q[0]
        self.dynamic_tf_stamped_.transform.rotation.y = q[1]
        self.dynamic_tf_stamped_.transform.rotation.z = q[2]
        self.dynamic_tf_stamped_.transform.rotation.w = q[3]
        # Esta seccion realizaria la rotacion de un frame por una catidad de bits especifico

        # Se envia los valores de transformaciones
        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_tf_stamped_)

        # Finalmente se actualiza el punto final de los frames
        self.last_x_ = self.dynamic_tf_stamped_.transform.translation.x

        # Finalmente se actualiza el valor de cambios en la variable de memoria de rotacion
        self.rotations_counter_ += 1
        self.last_orientation_ = q

        # Validacion de llegar a posicion limite, para cambiar la polaridad del movimiento del frame
        if self.rotations_counter_ >= 100:
            # Se realizaria la matriz invertida del cambio de orientacion
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
            # Tambien es necesario reiniciar el valor de conteo de rotaciones
            self.rotations_counter_ = 0

    # Secrea la funcion de callback para el servicio de la matriz de transformacion
    def getTransformCallback(self, req, res):
        # Validacion de ejecucion de callback
        self.get_logger().info(f"La transformada calculada entre los frames {req.frame_id} y {req.frame_child}")

        # Se crea la variable que obtiene el valor solicitado (rquest)
        request_transform = TransformStamped()

        # TODO: Que hace esta pequeña párte
        # Segun entiendo, considera una funcion necesaria para conocer el cambio entre 2 frames de referencia
        try: # Se debe validar que no se ejecute un error al invocar esta funcion
            # Se asigna el valor obtenido a la variable
            request_transform = self.buffer_.lookup_transform(req.frame_id, req.frame_child, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("Ocurrio un error al intentar realizar la publicacion de frames usando el servidor")
            res.success = False # Se asigna el valor negativo al error
            return res

        # En caso que el servidor de la matriz de transformacion funcione correctamente
        res.transform = request_transform
        res.success = True # Se asigna el valor positivo

        return res


# Funcion de main que ejecuta el nodo creado anteriormente
def main(args = None):#
    # Lo primero es inicializar la libreria de ros
    rclpy.init(args = args)

    # Segundo la creacion del nodo de ejecucion
    node_ejecucion = nodo_transformation()
    # Tercero se ejecuta el spin del nodod creado
    rclpy.spin(node_ejecucion)

    # Se debe destruir el nodo al acabar
    node_ejecucion.destroy_node()
    # Finalmente apagar la libreria
    rclpy.shutdown()

# Para dar ejecucion con si fuera un nodo de ros2 
if __name__ == "__main__":
    main()