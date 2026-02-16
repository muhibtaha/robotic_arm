import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from .roboclaw_3 import Roboclaw

class RoverDriver(Node):
    def __init__(self):
        super().__init__('rover_driver_node')

        # --- 1. BAGLANTI AYARLARI ---
        self.port = "/dev/ttyUSB0"
        self.baudrate = 115200
        
        # --- 2. ADRES DEFTERİ (Buldugumuz Harita) ---
        # 128: M1 = Motor4(BilekSol), M2 = Motor3(Dirsek)
        # 129: M1 = Motor2(Omuz), M2 = Motor5(BilekSag/Kiskac)
        self.ADDR_128 = 0x80  
        self.ADDR_129 = 0x81 

        self.get_logger().info(f"Roboclaw Portu: {self.port}")

        # Roboclaw Nesnesini Baslat
        self.rc = Roboclaw(self.port, self.baudrate)
        if self.rc.Open():
            self.get_logger().info("✅ Roboclaw CONNECTED! (Daisy Chain Mode)")
        else:
            self.get_logger().error("❌ Roboclaw CONNECTION FAILED!")

        # --- 3. ABONELIK ---
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_komutlari',
            self.listener_callback,
            10)

        # --- 4. RAMPALAMA DEGISKENLERI ---
        # Hedeflenen hizlar (Joystickten gelen)
        self.targets = [0.0] * 7
        # Motora o an gonderilen hizlar (Yavas yavas artacak)
        self.currents = [0.0] * 7
        
        # RAMP ADIMI: Her 0.05 saniyede hizi 2000 arttirdim. Daha yumusak istersek (1000), daha seri istersek (4000) yapabiliriz. Deneyince güncellenecek!
        self.RAMP_STEP = 2000.0

        # Saniyede 20 kere (0.05s) control_loop fonksiyonunu calistir
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Driver Ready. Soft Start (Ramping) Active.")

    def listener_callback(self, msg):
        # Gelen veriyi hafizaya al
        if len(msg.data) >= 7:
            self.targets = msg.data

    # Hesaplanan rampali hizi motora gonderir.
    def drive_raw(self, address, motor_num, pwm_value):
        ROBOCLAW_MAX = 32767
        duty = int(pwm_value)
        
        # Guvenlik Sinirlamasi (Clamp)
        if duty > ROBOCLAW_MAX: duty = ROBOCLAW_MAX
        if duty < -ROBOCLAW_MAX: duty = -ROBOCLAW_MAX
        
        try:
            if motor_num == 1: 
                self.rc.DutyM1(address, duty)
            elif motor_num == 2: 
                self.rc.DutyM2(address, duty)
        except Exception as e:
            self.get_logger().error(f"⚠️ Motor Error (Addr: {address}, M{motor_num}): {e}")

    # Bu fonksiyon surekli calisir ve hizi hedefe dogru YAVASCA yaklastirir.
    def control_loop(self):
        # Sadece Kol Motorlari ile ilgileniyoruz (Index 2, 3, 4, 5)
        for i in range(2, 6):
            target = self.targets[i]
            current = self.currents[i]
            
            error = target - current
            
            # Eger fark cok azsa direkt hedefe esitle (Titremeyi onle)
            if abs(error) < self.RAMP_STEP:
                self.currents[i] = target
            else:
                # Fark buyukse, RAMP_STEP kadar artir veya azalt
                if error > 0:
                    self.currents[i] += self.RAMP_STEP
                else:
                    self.currents[i] -= self.RAMP_STEP

        # --- GUNCEL HIZLARI MOTORLARA GONDER ---
        
        # 1. OMUZ (Index 2) -> Adres 129, Motor 1
        self.drive_raw(self.ADDR_129, 1, self.currents[2])

        # 2. DIRSEK (Index 3) -> Adres 128, Motor 2
        self.drive_raw(self.ADDR_128, 2, self.currents[3])

        # 3. BILEK SOL (Index 4) -> Adres 128, Motor 1
        self.drive_raw(self.ADDR_128, 1, self.currents[4])

        # 4. KISKAC / BILEK SAG (Index 5) -> Adres 129, Motor 2
        self.drive_raw(self.ADDR_129, 2, self.currents[5])

    # Kapanista her seyi durdur.
    def stop_all(self):
        self.get_logger().info("Stopping all motors...")
        self.drive_raw(self.ADDR_128, 1, 0)
        self.drive_raw(self.ADDR_128, 2, 0)
        self.drive_raw(self.ADDR_129, 1, 0)
        self.drive_raw(self.ADDR_129, 2, 0)

def main(args=None):
    rclpy.init(args=args)
    node = RoverDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
