import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy              
from std_msgs.msg import Float32MultiArray   

class RoverTeleop(Node):
    def __init__(self):
        super().__init__('rover_teleop_node')

        # Indeks Haritasi:
        # 0:Bos, 1:Bos, 2:M1, 3:M2, 4:M3, 5:M4, 6:M5, 7:M6, 8:Gripper
        self.last_commands = [0.0] * 9

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'motor_komutlari',
            10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("OPERATOR HAZIR! Gripper analogda, gerisi butonlarda.")

    def joy_callback(self, msg):
        emirler = Float32MultiArray()
        emirler.data = [0.0] * 9 

        # --- A. ANALOG EKSENLER (GRIPPER VE 1. MOTOR) ---
        
        # Gripper / Kıskaç (axes[1] - Joystick İleri/Geri)
        if msg.axes[1] > 0.5:
            emirler.data[8] = 30000.0
        elif msg.axes[1] < -0.5:
            emirler.data[8] = -30000.0

        # 1. Motor (axes[0] - Joystick Sağ/Sol)
        # (Tuş sayımız 11 olduğu için bu motoru mecburen eksene aldık)
        if msg.axes[0] > 0.5:
            emirler.data[2] = 30000.0
        elif msg.axes[0] < -0.5:
            emirler.data[2] = -30000.0


        # --- B. BUTONLAR (GERİYE KALAN 5 MOTOR) ---

        # 2. Motor (Buton 0 ve 1)
        if msg.buttons[0] == 1:
            emirler.data[3] = 30000.0
        elif msg.buttons[1] == 1:
            emirler.data[3] = -30000.0

        # 3. Motor (Buton 2 ve 3)
        if msg.buttons[2] == 1:
            emirler.data[4] = 30000.0
        elif msg.buttons[3] == 1:
            emirler.data[4] = -30000.0

        # 4. Motor (Buton 4 ve 5)
        if msg.buttons[4] == 1:
            emirler.data[5] = 30000.0
        elif msg.buttons[5] == 1:
            emirler.data[5] = -30000.0

        # 5. Motor (Buton 6 ve 7)
        if msg.buttons[6] == 1:
            emirler.data[6] = 30000.0
        elif msg.buttons[7] == 1:
            emirler.data[6] = -30000.0

        # 6. Motor (Buton 8 ve 9)
        if msg.buttons[8] == 1:
            emirler.data[7] = 30000.0
        elif msg.buttons[9] == 1:
            emirler.data[7] = -30000.0

        # --- C. Hafızayı Güncelle ve Yayınla ---
        self.last_commands = emirler.data
        self.publisher_.publish(emirler)
    
    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.last_commands
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
