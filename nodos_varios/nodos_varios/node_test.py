#!/usr/bin/env python3
# Seccion de importe de librerias
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Seccion de clase de nodo
class NodoTest(Node):
    def __init__(self) -> None:
        super().__init__("Nodo_comunicacion_driver")
        self.velocidad: int = 128
        self.variationValue: float = 128

        # Inicialization of subs
        self._init_subscribers()

    def _init_subscribers(self):
        self.subs_velChange = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback_vel,
            10
        )
        self.subs_velChange

    # Call backs for subscriber
    def callback_vel(self, msg: Twist):
        varVel:float = float(msg.linear.x)
        if self.velocidad + int(128*varVel) < 200 and self.velocidad + int(128*varVel) > 50: 
            self.velocidad = int(128 + 128*varVel)
            self.get_logger().info(str(self.velocidad))

def main(args = None):
    # Inicialization of rclpy
    rclpy.init(args = args)

    # Create object node
    cmd_control = NodoTest()
    rclpy.spin(cmd_control)

    # End of Process of Node
    cmd_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
