#This code combines both obstacle.py and imu.py into one file
#This allows the pi to get depth sensor and imu data simultanisly
import time
import math
import threading
import board
import busio
import RPi.GPIO as GPIO
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from VL53L1X import VL53L1X

# === IMU Code ===
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
                yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

                # Pitch - Rotation about X-axis (correct formula)
                pitch = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))

                # Roll - Rotation about Y-axis
                roll = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))

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


# === Depth Sensor Code ===
# Configuration
XSHUT_PINS = [6, 5]  # BCM GPIO pins
TIMING_BUDGET = 20  # 20ms → 50Hz (minimum stable value)
INTER_MEASUREMENT = 20  # Must be ≥ timing budget
RANGE_MODE = 1  # 1 = Short range (fastest)
I2C_BUS = 1  # Raspberry Pi I2C bus
BASE_ADDRESS = 0x29  # Default VL53L1X address

# Thread-safe resources
i2c_lock = threading.Lock()


class SensorManager:
    def __init__(self):
        self.sensors = []
        self.threads = []
        self.running = True
        self.distances = [0] * len(XSHUT_PINS)  # Shared distance data
        self._init_hardware()

    def _init_hardware(self):
        """Single-threaded hardware initialization"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Power cycle all sensors
        for pin in XSHUT_PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        time.sleep(0.1)

        # Initialize sensors sequentially
        for idx, pin in enumerate(XSHUT_PINS):
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.05)  # Reduced power-up delay

            address = BASE_ADDRESS + idx
            if idx > 0:
                with i2c_lock:
                    temp = VL53L1X(I2C_BUS, BASE_ADDRESS)
                    temp.open()
                    temp.set_i2c_address(address)
                    temp.close()
                    time.sleep(0.01)

            with i2c_lock:
                sensor = VL53L1X(I2C_BUS, address)
                sensor.open()
                sensor.set_timing(TIMING_BUDGET, INTER_MEASUREMENT)
                sensor.set_range_mode(RANGE_MODE)
                sensor.start_ranging(RANGE_MODE)
                self.sensors.append(sensor)

    def _read_worker(self, sensor_id):
        """Thread-safe sensor reading"""
        sensor = self.sensors[sensor_id]
        while self.running:
            if sensor.data_ready:
                with i2c_lock:
                    self.distances[sensor_id] = sensor.get_distance()
                    sensor.clear_interrupt()
            time.sleep(0.001)  # Minimal sleep

    def start(self):
        """Start measurement threads"""
        for idx in range(len(self.sensors)):
            thread = threading.Thread(
                target=self._read_worker, args=(idx,), daemon=True
            )
            thread.start()
            self.threads.append(thread)

    def stop(self):
        """Clean shutdown"""
        self.running = False
        for sensor in self.sensors:
            with i2c_lock:
                sensor.stop_ranging()
                sensor.close()
        GPIO.cleanup()


# === Main Program ===
if __name__ == "__main__":
    # Initialize IMU and Depth Sensor
    imu = IMUThread()
    depth_sensor_manager = SensorManager()

    # Start threads
    imu.start()
    depth_sensor_manager.start()

    try:
        while True:
            # Read IMU data
            yaw, pitch, roll = imu.get_angles()
            print(f"\033[F\033[K Yaw: {yaw:.2f}° | Pitch: {pitch:.2f}° | Roll: {roll:.2f}°")

            # Read Depth Sensor data
            for idx, dist in enumerate(depth_sensor_manager.distances):
                print(f"Sensor {idx + 1}: {dist:4} mm")

            # Clear previous output
            print("\033[F" * (len(depth_sensor_manager.sensors) + 2))

            # Update at 50Hz
            time.sleep(0.02)

    except KeyboardInterrupt:
        # Clean shutdown
        imu.stop()
        depth_sensor_manager.stop()
        imu.join()
        print("\nStopped cleanly")
