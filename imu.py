import time
import board
import busio
import math
from adafruit_bno08x import BNO08X_I2C

# Initialize I2C bus and sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor = BNO08X_I2C(i2c)

def quaternion_to_euler(w, x, y, z):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    Roll is rotation around the x-axis, pitch is rotation around the y-axis, and yaw is rotation around the z-axis.
    Returns angles in radians.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    # Use 90 degrees if out of range
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

while True:
    # Retrieve quaternion from the sensor (format: (w, x, y, z))
    quat = sensor.quaternion
    if quat is not None:
        w, x, y, z = quat
        roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
        # Convert radians to degrees for easier interpretation
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        print("Roll: {:.2f}°, Pitch: {:.2f}°, Yaw: {:.2f}°".format(roll_deg, pitch_deg, yaw_deg))
    else:
        print("Quaternion data not available.")
    
    time.sleep(0.1)
