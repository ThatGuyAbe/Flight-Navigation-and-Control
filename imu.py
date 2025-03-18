import time
import math
import threading
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

class IMUThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.running = True
        self.lock = threading.Lock()
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self._setup_sensor()

    def _setup_sensor(self):
        """One-time sensor initialization (runs in main thread)"""
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(self.i2c)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.5)  # Critical for sensor init

    def run(self):
        """Continuous sensor reading thread"""
        while self.running:
            try:
                quat = self.bno.quaternion
                if quat is None or len(quat) < 4:
                    continue

                # Unpack with validation
                x, y, z, w = quat[:4]

                # Compute angles
                # Yaw (azimuth) - Rotation about Z-axis
                yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))

                # Pitch - Rotation about X-axis (correct formula)
                pitch = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))

                # Roll - Rotation about Y-axis
                roll = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
                # Update shared values atomically
                with self.lock:
                    self.yaw = math.degrees(yaw)
                    self.pitch = math.degrees(pitch)
                    self.roll = math.degrees(roll)

            except (ValueError, IndexError) as e:
                print(f"Sensor error: {e}")
                time.sleep(0.01)

    def get_angles(self):
        """Thread-safe angle retrieval"""
        with self.lock:
            return self.yaw, self.pitch, self.roll

    def stop(self):
        self.running = False
        self.i2c.deinit()

# Usage
if __name__ == "__main__":
    imu = IMUThread()
    imu.start()

    try:
        while True:
            yaw, pitch, roll = imu.get_angles()
            print(f"\033[F\033[K Yaw: {yaw:.2f}° | Pitch: {pitch:.2f}° | Roll: {roll:.2f}°")
            time.sleep(0.02)  # 50Hz display update
    except KeyboardInterrupt:
        imu.stop()
        imu.join()
        print("\nStopped cleanly")
