import time
import RPi.GPIO as GPIO
from VL53L1X import VL53L1X

tb = 200
im = 200
rm = 1 # 1 = short, 2 = medium, 3 = long

# Define XSHUT pins (BCM numbering)
xshut_pins = [6, 5]  # Example: GPIO6 and GPIO5

# Setup GPIO
GPIO.setmode(GPIO.BCM)
for pin in xshut_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

sensors = []

for i, pin in enumerate(xshut_pins):
    # Power on the current sensor
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.5)  # Allow sensor to initialize

    # Initialize sensor at default address 0x29
    sensor = VL53L1X(i2c_bus=1, i2c_address=0x29)
    sensor.open()

    if i != 0:
        new_address = 0x29 + i
        sensor.set_i2c_address(new_address)
        # Reinitialize sensor object with new address
        sensor = VL53L1X(i2c_bus=1, i2c_address=new_address)
        sensor.open()

    sensor.set_timing(tb, im)
    sensor.start_ranging(rm)
    sensors.append(sensor)

try:
    while True:
        for i, sensor in enumerate(sensors):
            print(f"Sensor {i+1}: {sensor.get_distance()} mm")
        time.sleep(0.1)
except KeyboardInterrupt:
    for sensor in sensors:
        sensor.stop_ranging()
    GPIO.cleanup()
