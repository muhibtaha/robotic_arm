#sudo chmod 666 /dev/ttyUSB0 --> USB bağlanması için terminale yazılması gereken izin kodu

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from .roboclaw_3 import Roboclaw
import time

class RoverDriver(Node):
    def __init__(self):
        super().__init__('rover_driver_node')

        # --- AYARLAR ---
        self.port = "/dev/ttyUSB0"
        self.baudrate = 115200
        
        # SÜRÜCÜ ADRESLERİ
        self.ADDR_128 = 0x80  
        self.ADDR_129 = 0x81 
        self.ADDR_130 = 0x82  
        self.ADDR_131 = 0x83  # YENİ EKLENEN GRİPPER KARTI

        self.get_logger().info(f"Port: {self.port} | Baud: {self.baudrate}")

        # --- BAGLANTIYI BASLAT ---
        self.connect_roboclaw()

        # --- ABONELIK ---
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_komutlari',
            self.listener_callback,
            10)

        # --- DEGISKENLER ---
        # İndeks Haritası (Teleop ile Birebir Aynı):
        # 0:Bos, 1:Bos, 2:M1, 3:M2, 4:M3, 5:M4, 6:M5, 7:M6, 8:Gripper
        self.targets = [0.0] * 9
        self.currents = [0.0] * 9
        self.RAMP_STEP = 2000.0
        self.last_msg_time = time.time()
        self.TIMEOUT_SEC = 0.5

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Driver Started (4 Controllers Aktif)")

    def connect_roboclaw(self):
        try:
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
                self.get_logger().error("❌ Connection Failed! (Kabloyu kontrol et)")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Connection Error: {e}")
            return False

    def listener_callback(self, msg):
        if len(msg.data) >= 9:
            for i in range(9):
                self.targets[i] = msg.data[i]
            self.last_msg_time = time.time()

    def control_loop(self):
        # --- WATCHDOG (Baglanti koparsa durdur) ---
        elapsed_time = time.time() - self.last_msg_time
        if elapsed_time > self.TIMEOUT_SEC:
            self.targets = [0.0] * 9

        # --- RAMPALAMA (Ani hareketleri yumuşat) ---
        for i in range(2, 9):
            target = self.targets[i]
            current = self.currents[i]
            error = target - current
            
            if abs(error) < self.RAMP_STEP:
                self.currents[i] = target
            else:
                if error > 0: self.currents[i] += self.RAMP_STEP
                else: self.currents[i] -= self.RAMP_STEP

        # --- MOTORLARI SUR ---
        try:
            ROBOCLAW_MAX = 32767
            
            # Güvenli Sınırlandırma (Değerleri al ve Max limiti aşmasını engelle)
            v_m1 = max(min(int(self.currents[2]), ROBOCLAW_MAX), -ROBOCLAW_MAX)
            v_m2 = max(min(int(self.currents[3]), ROBOCLAW_MAX), -ROBOCLAW_MAX)
            v_m3 = max(min(int(self.currents[4]), ROBOCLAW_MAX), -ROBOCLAW_MAX)
            v_m4 = max(min(int(self.currents[5]), ROBOCLAW_MAX), -ROBOCLAW_MAX)
            v_m5 = max(min(int(self.currents[6]), ROBOCLAW_MAX), -ROBOCLAW_MAX)
            v_m6 = max(min(int(self.currents[7]), ROBOCLAW_MAX), -ROBOCLAW_MAX)
            v_grip = max(min(int(self.currents[8]), ROBOCLAW_MAX), -ROBOCLAW_MAX)
            
            # --- YENİ BAĞLANTI HARİTASI (WhatsApp'tan gelen listeye göre) ---
            
            # 128 -> M1: 4. Motor | M2: 1. Motor
            self.rc.DutyM1M2(self.ADDR_128, v_m4, v_m1)
            time.sleep(0.01) 

            # 129 -> M1: 2. Motor | M2: 3. Motor
            self.rc.DutyM1M2(self.ADDR_129, v_m2, v_m3)
            time.sleep(0.01)

            # 130 -> M1: 5. Motor | M2: 6. Motor
            self.rc.DutyM1M2(self.ADDR_130, v_m5, v_m6)
            time.sleep(0.01)

            # 131 -> M1: Gripper  | M2: Bos
            self.rc.DutyM1M2(self.ADDR_131, v_grip, 0)

        except Exception as e:
            self.get_logger().error(f"⚠️ USB ERROR: {e}. Trying to Reconnect...")
            self.connect_roboclaw()

    def stop_all(self):
        try:
            self.rc.DutyM1M2(self.ADDR_128, 0, 0)
            time.sleep(0.02)
            self.rc.DutyM1M2(self.ADDR_129, 0, 0)
            time.sleep(0.02)
            self.rc.DutyM1M2(self.ADDR_130, 0, 0)
            time.sleep(0.02)
            self.rc.DutyM1M2(self.ADDR_131, 0, 0)
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
