import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy              # Joystick verisi (GİRDİ)
from std_msgs.msg import Float32MultiArray   # Motor emirleri (ÇIKTI)

class RoverTeleop(Node):
    def __init__(self):
        super().__init__('rover_teleop_node')

        # En son gonderilecek komutlari burada saklayacagiz
        # [SolTeker, SagTeker, Omuz, Dirsek, Bilek, Kiskac, Yedek]
        self.last_commands = [0.0] * 7

        # 1. GİRİŞ: Joystick'i dinle (/joy konusuna abone ol)
        # ROS'un "joy" paketi joystick verisini buraya atacak.
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
        # Joystick'e dokunmasan bile surucunun baglantiyi koparmamasi icin
        # saniyede 10 kere (0.1s) son komutu tekrar tekrar gonderiyoruz.
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("OPERATOR HAZIR! Joystick verisi bekleniyor...")

    def joy_callback(self, msg):
        # Bu fonksiyon, Joystick'e her dokunduğunda otomatik çalışır.

        # --- A. Boş bir emir listesi oluştur (7 motor için) ---
        emirler = Float32MultiArray()
        # [SolTeker, SagTeker, Omuz, Dirsek, Bilek, Kıskaç, Yedek]
        emirler.data = [0.0] * 7 

        # --- B. Joystick Tuşlarını Motora Çevir ---

        # 1. SÜRÜŞ (Joystick Eksenleri)
        # Genelde Sol Analog (Axis 1: İleri/Geri, Axis 0: Sağ/Sol)
        axis_ileri = msg.axes[1] 
        axis_donus = msg.axes[0] 
        
        # Deadzone (Ufak titremeleri yoksay)
        if abs(axis_ileri) < 0.1: axis_ileri = 0.0
        if abs(axis_donus) < 0.1: axis_donus = 0.0

        # Tank Sürüşü Mantığı (Basitçe)
        emirler.data[0] = (axis_ileri * 15000.0)   # Sol Teker
        emirler.data[1] = (axis_ileri * 15000.0)   # Sağ Teker
        # (Dönüş mantığını şimdilik basit tuttum, sonra geliştiririz)

        # 2. ROBOT KOL (Butonlar)
        # NOT: Buton numaraları Joystick modeline göre değişebilir!
        # Deneyerek bulacağız: buttons[0] genelde "X" veya "A" tuşudur.
        
        # Omuz Motoru (Buton 0 ve 1)
        if msg.buttons[0] == 1:   
            emirler.data[2] = 34000.0
        elif msg.buttons[1] == 1: 
            emirler.data[2] = -34000.0
        
        # Dirsek Motoru (Buton 2 ve 3)
        if msg.buttons[2] == 1:
            emirler.data[3] = 20000.0
        elif msg.buttons[3] == 1:
            emirler.data[3] = -20000.0

        # Bilek Motoru (Buton 4 ve 5)
        if msg.buttons[4] == 1:
            emirler.data[4] = 35000.0
        elif msg.buttons[5] == 1:
            emirler.data[4] = -34000.0

        # Kıskaç (Buton 6 ve 7)
        if msg.buttons[6] == 1:
            emirler.data[5] = -30000.0
        elif msg.buttons[7] == 1:
            emirler.data[5] = 30000.0

        # Hafizayi güncelle.
        self.last_commands = emirler.data
        # --- C. Listeyi Yayınla (Kargoyu Gönder) ---
        self.publisher_.publish(emirler)
    
    # Saniyede 10 kere calisir ve son durumu Driver'a bildirir.
    # Bu sayede Driver'in Watchdog'u (Güvenlik kilidi) devreye girmez.
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

