import time
import threading
import RPi.GPIO as GPIO
from VL53L1X import VL53L1X

# Define XSHUT pins for multiple sensors (using BCM numbering)
xshut_pins = [6, 5]  # Example GPIO pins connected to XSHUT

# Sensor configuration
timing_budget = 200
intermeasurement = 200
ranging_mode = 1  # 1 = short, 2 = medium, 3 = long

# Setup GPIO
GPIO.setmode(GPIO.BCM)
for pin in xshut_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)  # Initially disable all sensors

# Shared list to hold sensor objects
sensors = []

# Function for reading distance from each sensor in a separate thread
def read_sensor(sensor, sensor_id):
    while True:
        distance = sensor.get_distance()
        print(f"Sensor {sensor_id}: {distance} mm")
        time.sleep(0.1)  # Adjust polling rate for performance

# Initialize each sensor
threads = []
for i, pin in enumerate(xshut_pins):
    # Power on each sensor one at a time
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.5)  # Give sensor time to initialize

    # Initialize sensor at default address
    sensor = VL53L1X(i2c_bus=1, i2c_address=0x29)
    sensor.open()

    # Change the I2C address if it's not the first sensor
    if i != 0:
        new_address = 0x29 + i
        sensor.set_i2c_address(new_address)
        # Reinitialize sensor with new address
        sensor = VL53L1X(i2c_bus=1, i2c_address=new_address)
        sensor.open()

    # Set sensor parameters
    sensor.set_timing(timing_budget, intermeasurement)
    sensor.start_ranging(ranging_mode)

    # Add sensor to the list
    sensors.append(sensor)

    # Start a new thread for each sensor
    thread = threading.Thread(target=read_sensor, args=(sensor, i + 1), daemon=True)
    threads.append(thread)
    thread.start()

# Keep the main thread alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping threads and cleaning up GPIO...")
    for sensor in sensors:
        sensor.stop_ranging()
    GPIO.cleanup()

