# Seccion de importe de librerias
import rclpy
from rclpy.node import Node
# Seccion de librerias de interfaces necesarias
from turtlesim.msg import Pose
# Seccion de importe de librerias utilitarias
import math

# Seccion de clase de nodo, funciona para la ejecucion del nodo
class RotationMatrixTurtle(Node):
    def __init__(self) -> None:
        super().__init__("Nodo_calculo_rotation_matrix")

        # Primero se tiene que hacer la subscripcion a la posicion de cada una de las tortugas
        self.turtle_pose_sub_1 = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.turtle1PoseCallback,
            10
        )
        self.turtle_pose_sub_2 = self.create_subscription(
            Pose,
            "/turtle2/pose",
            self.turtle2PoseCallback,
            10
        )

        self.pose_actualizada_tortuga_1: Pose = Pose()
        self.pose_actualizada_tortuga_2: Pose = Pose()
    
    # Seccion de funciones callbacks de subscriptores
    def turtle1PoseCallback(self, msg):
        # Se asigna el nuevo valor de posicion en la variable de instancia de pose
        self.pose_actualizada_tortuga_1 = msg

    def turtle2PoseCallback(self, msg):
        # Asignacion de valor a variable
        self.pose_actualizada_tortuga_2 = msg

        # Calculo de matriz de translacion
        Tx = self.pose_actualizada_tortuga_2.x - self.pose_actualizada_tortuga_1.x
        Ty = self.pose_actualizada_tortuga_2.y - self.pose_actualizada_tortuga_1.y
        
        # Calculo de matriz de rotacion
        theta_rad = self.pose_actualizada_tortuga_2.theta - self.pose_actualizada_tortuga_1.theta
        theta_deg = 180*theta_rad/math.pi
        
        # Calculo de cada componente de la matrix
        comp_r11 = math.cos(theta_rad)
        comp_r12 = -math.sin(theta_rad)
        comp_r21 = math.sin(theta_rad)
        comp_r22 = math.cos(theta_rad)

        # visualizacion de valores calculados
        self.get_logger().info(f"""\n
            Vector de transalacion tortuga 2 vs tortuga 1: \n
            Tx: {round(Tx, 2)} \n
            Ty: {round(Ty, 2)} \n
            Vector de la matriz de rotacion, referencia de posicion entre tortuga 2 vs tortuga 1 \n
            theta_rad: {round(theta_rad, 2)} \n
            theta_deg: {round(theta_deg, 2)} \n
            | R11            R12 | : | {round(comp_r11, 2)}           {round(comp_r12, 2)} |
            | R21            R22 | : | {round(comp_r21, 2)}           {round(comp_r22, 2)} |
            """)

# Seccion de funcion main
def main(args = None):
    # Inicializacion de libreria
    rclpy.init(args = args)

    # Creacion y ejecucion de nodo
    objeto_node = RotationMatrixTurtle()
    rclpy.spin(objeto_node)

    # Destruccion de nodo
    objeto_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()