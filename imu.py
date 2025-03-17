import time
import math
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize BNO085
bno = BNO08X_I2C(i2c)

# Enable rotation vector feature
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
time.sleep(0.5)  # Allow sensor to initialize

def get_yaw_pitch_roll():
    try:
        quat = bno.quaternion

        # **Fix: Check if quaternion data is valid before using it**
        if quat is None or len(quat) < 4:
            print("Warning: Quaternion data is incomplete or unavailable")
            return None, None, None  # Return None to avoid an error

        quat_i, quat_j, quat_k, quat_real = quat

        # Compute yaw, pitch, roll
        yaw = math.atan2(2.0 * (quat_real * quat_k + quat_i * quat_j),
                         1.0 - 2.0 * (quat_j**2 + quat_k**2))

        sinp = 2.0 * (quat_real * quat_j - quat_k * quat_i)
        sinp = max(-1.0, min(1.0, sinp))  # **Clamp the value to prevent math domain errors**
        pitch = math.asin(sinp)

        roll = math.atan2(2.0 * (quat_real * quat_i + quat_j * quat_k),
                          1.0 - 2.0 * (quat_i**2 + quat_j**2))

        return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)

    except IndexError as e:
        print(f"IndexError: {e} - Quaternion data might be incomplete")
        return None, None, None  # Prevent the crash

try:
    while True:
        yaw, pitch, roll = get_yaw_pitch_roll()
        if yaw is not None:
            print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
        else:
            print("Skipping this reading due to missing data")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")

