import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
# from .roboclaw_3 import Roboclaw  <-- Kutuphane buraya gelecek

class RoverDriver(Node):
    def __init__(self):
        super().__init__('rover_driver_node')

        # TODO: BURAYA MOTOR BAGLANTILARINI YAP (roboclaw.Open vs.)
        # self.rc0 = ...

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_komutlari',
            self.listener_callback,
            10)

        # TODO: BURAYA RAMPALAMA MANTIGI ICIN DEGISKENLERI EKLE
        self.targets = [0.0] * 7
        self.currents = [0.0] * 7

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Surucu Baslatildi! (Henuz motorlar bagli degil)")

    def listener_callback(self, msg):
        # Gelen veriyi hafizaya al
        if len(msg.data) >= 7:
            self.targets = msg.data

    def control_loop(self):
        # TODO: BURAYA RAMPALAMA (Yumusak Kalkis) KODUNU YAZ
        # ...

        # TODO: BURAYA MOTOR SURME (rc.DutyM1) KODLARINI YAZ
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RoverDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
