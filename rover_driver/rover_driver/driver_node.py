import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from .roboclaw_3 import Roboclaw
import time # ZAMAN TAKIBI ICIN EKLENDI

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

        # --- 5. GUVENLIK (WATCHDOG) ---
        # En son ne zaman veri geldi?
        self.last_msg_time = time.time()
        # 0.5 saniye veri gelmezse robotu durdur
        self.TIMEOUT_SEC = 0.5

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
        # --- GUVENLIK KONTROLU (TIMEOUT) ---
        elapsed_time = time.time() - self.last_msg_time

        # Eger 0.5 saniyeden uzun suredir veri gelmiyorsa:
        if elapsed_time > self.TIMEOUT_SEC:
            # Hedefleri SIFIRLA (Robotu durmaya zorla)
            self.targets = [0.0] * 7
            
            # Surekli log basmasin diye saniyede bir uyar
            if int(elapsed_time) % 2 == 0: 
                self.get_logger().warn(f"⚠️ SIGNAL LOST! Stopping motors... ({elapsed_time:.1f}s)", throttle_duration_sec=1.0)
        
        # --- RAMPALAMA ISLEMI ---
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
