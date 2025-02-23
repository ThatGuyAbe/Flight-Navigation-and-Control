import time
import smbus
import VL53L1X
import RPi.GPIO as GPIO

# GPIO pins connected to the XSHUT pins of the sensors
XSHUT_PINS = [4, 17, 27, 22, 23]  # Replace with your GPIO pins
NUM_SENSORS = len(XSHUT_PINS)

# I2C addresses for the sensors
I2C_ADDRESSES = [0x29, 0x30, 0x31, 0x32, 0x33]  # Unique addresses for each sensor

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
for pin in XSHUT_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)  # Shut down all sensors initially

# Initialize I2C bus
i2c_bus = smbus.SMBus(1)  # Use I2C bus 1 (GPIO 2 and 3)

# Function to set the I2C address of a sensor
def set_sensor_address(xshut_pin, address):
    GPIO.output(xshut_pin, GPIO.HIGH)  # Enable the sensor
    time.sleep(0.1)  # Wait for the sensor to power up
    sensor = VL53L1X.VL53L1X(i2c_bus=i2c_bus)
    sensor.open()  # Initialize the sensor
    sensor.change_address(address)  # Change the I2C address
    sensor.close()  # Close the sensor
    time.sleep(0.1)

# Set unique I2C addresses for all sensors
for i in range(NUM_SENSORS):
    set_sensor_address(XSHUT_PINS[i], I2C_ADDRESSES[i])
    print(f"Sensor {i} initialized with address {hex(I2C_ADDRESSES[i])}.")

# Initialize sensors with their new addresses
sensors = []
for i in range(NUM_SENSORS):
    sensor = VL53L1X.VL53L1X(i2c_bus=i2c_bus, i2c_address=I2C_ADDRESSES[i])
    sensor.open()  # Initialize the sensor
    sensor.start_ranging()  # Start ranging
    sensors.append(sensor)
    print(f"Sensor {i} started ranging.")

# Main loop to read distance from all sensors
try:
    while True:
        for i in range(NUM_SENSORS):
            distance = sensors[i].get_distance()  # Get distance in mm
            print(f"Sensor {i}: {distance} mm")
        print("-------------------")
        time.sleep(1)  # Wait 1 second before the next reading
except KeyboardInterrupt:
    print("Exiting...")
finally:
    for sensor in sensors:
        sensor.stop_ranging()  # Stop ranging for all sensors
    GPIO.cleanup()  # Clean up GPIO