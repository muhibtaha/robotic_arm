#sudo chmod 666 /dev/ttyUSB0 --> USB bağlanması için terminale yazılması gereken izin kodu

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from .roboclaw_3 import Roboclaw
import time
import serial

class RoverDriver(Node):
    def __init__(self):
        super().__init__('rover_driver_node')

        # --- AYARLAR ---
        self.port = "/dev/ttyUSB0"
        self.baudrate = 115200
        
        # SÜRÜCÜ ADRESLERİ
        self.ADDR_128 = 0x80  
        self.ADDR_129 = 0x81 
        self.ADDR_130 = 0x82  # YENİ EKLENEN KART

        self.get_logger().info(f"Port: {self.port} | Baud: {self.baudrate}")

        # --- BAGLANTIYI BASLAT ---
        self.connect_roboclaw()

        # --- ABONELIK ---
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_komutlari',
            self.listener_callback,
            10)

        # --- DEGISKENLER (Kapasite 9'a Çıkarıldı) ---
        self.targets = [0.0] * 9
        self.currents = [0.0] * 9
        self.RAMP_STEP = 2000.0
        self.last_msg_time = time.time()
        self.TIMEOUT_SEC = 0.5

        # FREKANSI DUSURUYORUZ (USB Rahatlasin diye 0.1s)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Driver Started (Robust Mode - 3 Controllers)")

    # Baglantiyi guvenli sekilde acar/yeniler
    def connect_roboclaw(self):
        try:
            # Eger eski baglanti varsa kapat (_port hatasi korumasi)
            if hasattr(self, 'rc'):
                if hasattr(self.rc, '_port') and self.rc._port is not None:
                    if self.rc._port.is_open:
                        self.rc._port.close()
            
            time.sleep(0.5)
            
            self.rc = Roboclaw(self.port, self.baudrate)
            if self.rc.Open():
                self.get_logger().info("✅ Roboclaw CONNECTED/RECONNECTED!")
                return True
            else:
                self.get_logger().error("❌ Connection Failed! (Kabloyu veya chmod 666 iznini kontrol et)")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Connection Error: {e}")
            return False

    def listener_callback(self, msg):
        # Esnek okuma: Gelen dizideki eleman sayısı kadar oku
        if len(msg.data) > 0:
            for i in range(min(len(msg.data), len(self.targets))):
                self.targets[i] = msg.data[i]
            self.last_msg_time = time.time()

    def control_loop(self):
        # --- WATCHDOG ---
        elapsed_time = time.time() - self.last_msg_time
        if elapsed_time > self.TIMEOUT_SEC:
            self.targets = [0.0] * 9
            if int(elapsed_time) % 2 == 0: 
                self.get_logger().warn(f"Signal Lost ({elapsed_time:.1f}s)", throttle_duration_sec=2.0)

        # --- RAMPALAMA (Tüm Kol Motorları İçin: İndeks 2'den 7'ye kadar) ---
        for i in range(2, 8):
            target = self.targets[i]
            current = self.currents[i]
            error = target - current
            
            if abs(error) < self.RAMP_STEP:
                self.currents[i] = target
            else:
                if error > 0: self.currents[i] += self.RAMP_STEP
                else: self.currents[i] -= self.RAMP_STEP

        # --- MOTORLARI SUR (TRY-EXCEPT ILE KORUMALI) ---
        try:
            ROBOCLAW_MAX = 32767
            
            # 1. Kart (129)
            val_omuz = int(self.currents[2])
            val_kiskac = int(self.currents[5])
            val_omuz = max(min(val_omuz, ROBOCLAW_MAX), -ROBOCLAW_MAX)
            val_kiskac = max(min(val_kiskac, ROBOCLAW_MAX), -ROBOCLAW_MAX)
            
            self.rc.DutyM1M2(self.ADDR_129, val_omuz, val_kiskac)
            
            time.sleep(0.02) 

            # 2. Kart (128)
            val_bilek = int(self.currents[4])
            val_dirsek = int(self.currents[3])
            val_bilek = max(min(val_bilek, ROBOCLAW_MAX), -ROBOCLAW_MAX)
            val_dirsek = max(min(val_dirsek, ROBOCLAW_MAX), -ROBOCLAW_MAX)

            self.rc.DutyM1M2(self.ADDR_128, val_bilek, val_dirsek)
            
            time.sleep(0.02)

            # 3. Kart (130 - YENİ EKLENEN)
            val_yeni1 = int(self.currents[6]) # şuan boşta
            val_yeni2 = int(self.currents[7])
            val_yeni1 = max(min(val_yeni1, ROBOCLAW_MAX), -ROBOCLAW_MAX)
            val_yeni2 = max(min(val_yeni2, ROBOCLAW_MAX), -ROBOCLAW_MAX)

            # --- AJAN KOD (Bunu Ekle) ---
            if abs(val_yeni1) > 1000:
                self.get_logger().info(f"🚀 130 ADRESİNE GÜÇ GİDİYOR: {val_yeni1}")

            self.rc.DutyM1M2(self.ADDR_130, val_yeni1, val_yeni2)

        except Exception as e:
            # HATA OLURSA (-32 Failed vb.) BURAYA DUSER
            self.get_logger().error(f"⚠️ USB ERROR: {e}. Trying to Reconnect...")
            # Baglantiyi yenile
            self.connect_roboclaw()

    def stop_all(self):
        try:
            self.rc.DutyM1M2(self.ADDR_128, 0, 0)
            time.sleep(0.02)
            self.rc.DutyM1M2(self.ADDR_129, 0, 0)
            time.sleep(0.02)
            self.rc.DutyM1M2(self.ADDR_130, 0, 0)
        except:
            pass

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
