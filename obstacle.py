import time
import threading
import RPi.GPIO as GPIO
from VL53L1X import VL53L1X

# Configuration
XSHUT_PINS = [6, 5]  # BCM GPIO pins
TIMING_BUDGET = 20    # 20ms → 50Hz (minimum stable value)
INTER_MEASUREMENT = 20  # Must be ≥ timing budget
RANGE_MODE = 1        # 1 = Short range (fastest)
I2C_BUS = 1           # Raspberry Pi I2C bus
BASE_ADDRESS = 0x29   # Default VL53L1X address

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
                target=self._read_worker,
                args=(idx,),
                daemon=True
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

# Main execution
if __name__ == "__main__":
    manager = SensorManager()
    manager.start()

    try:
        while True:
            # Display latest readings
            for idx, dist in enumerate(manager.distances):
                print(f"Sensor {idx+1}: {dist:4} mm")
            print("\033[F" * (len(manager.sensors) + 1))  # Clear previous output
            time.sleep(0.02)  # 50Hz display update
    except KeyboardInterrupt:
        manager.stop()
