import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy              # Joystick verisi (GİRDİ)
from std_msgs.msg import Float32MultiArray   # Motor emirleri (ÇIKTI)

class RoverTeleop(Node):
    def __init__(self):
        super().__init__('rover_teleop_node')

        # En son gonderilecek komutlari burada saklayacagiz
        # [0:SolTeker, 1:SagTeker, 2:Omuz, 3:Dirsek, 4:Bilek, 5:Kiskac, 6:YeniMotor1, 7:YeniMotor2, 8:Yedek]
        # Toplam kapasiteyi 9'a cikardik!
        self.last_commands = [0.0] * 9

        # 1. GİRİŞ: Joystick'i dinle (/joy konusuna abone ol)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        # 2. ÇIKIŞ: Motor emirlerini yayınla (/motor_komutlari konusuna yayın yap)
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'motor_komutlari',
            10)

        # --- TIMER ---
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("OPERATOR HAZIR! 3. Sürücü Aktif. Joystick verisi bekleniyor...")

    def joy_callback(self, msg):
        # --- A. Boş bir emir listesi oluştur (9 motor için) ---
        emirler = Float32MultiArray()
        emirler.data = [0.0] * 9 

        # --- B. Joystick Tuşlarını Motora Çevir ---

        # 1. SÜRÜŞ (Joystick Eksenleri)
        axis_ileri = msg.axes[1] 
        axis_donus = msg.axes[0] 
        
        # Deadzone (Ufak titremeleri yoksay)
        if abs(axis_ileri) < 0.1: axis_ileri = 0.0
        if abs(axis_donus) < 0.1: axis_donus = 0.0

        # Tank Sürüşü Mantığı
        emirler.data[0] = (axis_ileri * 15000.0)   # Sol Teker
        emirler.data[1] = (axis_ileri * 15000.0)   # Sağ Teker

        # 2. ROBOT KOL (Butonlar)
        
        # Omuz Motoru (Buton 1 ve 2) //MOTOR2
        if msg.buttons[1] == 1:   
            emirler.data[2] = 34000.0
        elif msg.buttons[2] == 1: 
            emirler.data[2] = -34000.0
        
        # Dirsek Motoru (Buton 3 ve 4) //MOTOR3
        if msg.buttons[3] == 1:
            emirler.data[3] = 20000.0
        elif msg.buttons[4] == 1:
            emirler.data[3] = -20000.0

        # Bilek Motoru (Buton 5 ve 6) //MOTOR4
        if msg.buttons[5] == 1:
            emirler.data[4] = 35000.0
        elif msg.buttons[6] == 1:
            emirler.data[4] = -34000.0

        # Kıskaç (Buton 7 ve 8) //MOTOR5
        if msg.buttons[7] == 1:
            emirler.data[5] = -30000.0
        elif msg.buttons[8] == 1:
            emirler.data[5] = 30000.0
            
        # --- YENİ EKLENEN KART (130 Adresi) --- //MOTOR6 
        # M2 kanalına bağlı 6. indeks çalışmıyor, 7. indeksle çalışıyor 
        if msg.buttons[9] == 1:
            emirler.data[7] = -30000.0
        elif msg.buttons[10] == 1:
            emirler.data[7] = 30000.0

        # Hafizayi güncelle.
        self.last_commands = emirler.data
        
        # --- C. Listeyi Yayınla ---
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
