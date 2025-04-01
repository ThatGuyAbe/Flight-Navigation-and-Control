import time
import math
import threading
import board
import busio
import RPi.GPIO as GPIO
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from VL53L1X import VL53L1X

# Configuration
TCA9548A_ADDRESS = 0x70
NUM_SENSORS = 5
IMU_CHANNEL = 0
SENSOR_CHANNELS = list(range(1, NUM_SENSORS + 1))
TIMING_BUDGET = 50
INTER_MEASUREMENT = 50
RANGE_MODE = 1
I2C_FREQUENCY = 100000  # Reduced to 100kHz for stability

# Thread-safe resources
i2c_lock = threading.Lock()

class TCA9548A:
    def __init__(self, i2c, address=TCA9548A_ADDRESS):
        self.i2c = i2c
        self.address = address
        self.current_channel = None
        
    def select_channel(self, channel):
        """Safely select channel with retries"""
        if channel == self.current_channel:
            return
            
        for attempt in range(3):
            try:
                with i2c_lock:
                    self.i2c.writeto(self.address, bytes([1 << channel]))
                    self.current_channel = channel
                    time.sleep(0.001)  # Critical delay
                    return
            except OSError as e:
                print(f"Channel select error (attempt {attempt+1}): {e}")
                time.sleep(0.01)
        raise RuntimeError(f"Failed to select channel {channel}")

class IMUThread(threading.Thread):
    def __init__(self, multiplexer):
        super().__init__()
        self.running = True
        self.multiplexer = multiplexer
        self.lock = threading.Lock()
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.bno = None
        self._init_sensor()

    def _init_sensor(self):
        """Initialize IMU through multiplexer"""
        self.multiplexer.select_channel(IMU_CHANNEL)
        with i2c_lock:
            self.bno = BNO08X_I2C(self.multiplexer.i2c, address=IMU_ADDRESS)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.5)

    def run(self):
        """Continuous IMU reading"""
        while self.running:
            try:
                self.multiplexer.select_channel(IMU_CHANNEL)
                with i2c_lock:
                    quat = self.bno.quaternion
                
                x, y, z, w = quat
                yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
                pitch = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
                roll = math.asin(2*(w*y - z*x))

                with self.lock:
                    self.yaw = math.degrees(yaw)
                    self.pitch = math.degrees(pitch)
                    self.roll = math.degrees(roll)

            except Exception as e:
                print(f"IMU error: {str(e)}")
                time.sleep(0.1)

    def get_angles(self):
        """Get current angles"""
        with self.lock:
            return self.yaw, self.pitch, self.roll

    def stop(self):
        self.running = False

class DepthSensorManager:
    def __init__(self):
        self.running = True
        self.sensors = []
        self.distances = [0] * len(SENSOR_CHANNELS)
        self._init_sensors()

    def _init_sensors(self):
        """Initialize depth sensors through multiplexer"""
        print("Initializing depth sensors...")
        for idx, channel in enumerate(SENSOR_CHANNELS):
            try:
                # Initialize sensor with multiplexer support
                sensor = VL53L1X(
                    i2c_bus=1, 
                    i2c_address=0x29,
                    tca9548a_num=channel,
                    tca9548a_addr=TCA9548A_ADDRESS
                )
                # The VL53L1X library handles the multiplexer internally
                # No need to manually call open() when using tca9548a parameters
                sensor.start_ranging(RANGE_MODE)
                self.sensors.append(sensor)
                print(f"Sensor {idx+1} on channel {channel} initialized")
            except Exception as e:
                print(f"Failed to initialize sensor {idx+1}: {str(e)}")
                self.sensors.append(None)

    def update_distances(self):
        """Update all sensor distances"""
        for i, sensor in enumerate(self.sensors):
            if sensor is None:
                self.distances[i] = -1
                continue
                
            try:
                self.distances[i] = sensor.get_distance()
            except Exception as e:
                print(f"Sensor {i+1} error: {str(e)}")
                self.distances[i] = -1

    def stop(self):
        """Clean shutdown"""
        self.running = False
        for sensor in self.sensors:
            if sensor is not None:
                try:
                    sensor.stop_ranging()
                except:
                    pass
def main():
    # Initialize I2C and multiplexer
    i2c = busio.I2C(board.SCL, board.SDA)
    multiplexer = TCA9548A(i2c)
    
    # Create sensor objects
    imu = IMUThread(multiplexer)
    depth_manager = DepthSensorManager()
    
    # Start IMU thread
    imu.start()
    
    # Signal handler for clean exit
    def signal_handler(sig, frame):
        print("\nShutting down...")
        imu.stop()
        depth_manager.stop()
        imu.join()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        while True:
            # Get IMU data
            yaw, pitch, roll = imu.get_angles()
            
            # Get depth sensor data
            depth_manager.update_distances()
            
            # Print formatted output
            print("\033[F\033[K" +  # Clear line
                  f"Yaw: {yaw:.1f}° | Pitch: {pitch:.1f}° | Roll: {roll:.1f}° | " +
                  " | ".join(f"Sensor{i+1}: {d}mm" 
                           for i, d in enumerate(depth_manager.distances)))
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        signal_handler(None, None)

if __name__ == "__main__":
    main()
