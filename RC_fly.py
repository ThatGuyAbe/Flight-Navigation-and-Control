import serial
import time

# MSP Command IDs
MSP_SET_ARM = 217  # Arm/disarm
MSP_SET_RAW_RC = 200  # Set raw RC channels

def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def send_msp_command(ser, command_id, data=[]):
    direction = ord('<')  # Outgoing direction indicator
    payload_length = len(data)
    message = [ord('$'), ord('M'), direction, payload_length, command_id] + data
    checksum = calculate_checksum(message[3:])  # XOR of payload_length, command and data bytes
    message.append(checksum)
    print("Sending MSP command:", " ".join(f"{byte:02X}" for byte in message))  # Debug: Show the outgoing message
    ser.write(bytearray(message))
    time.sleep(0.1)
    response = ser.read_all()
    if response:
        print("Response received:", response.hex())  # Debug: Show the response
    else:
        print("No response received.")

def set_arm(ser, arm):
    # Data: 1 to arm, 0 to disarm.
    data = [1] if arm else [0]
    send_msp_command(ser, MSP_SET_ARM, data)

def set_raw_rc(ser, roll=1500, pitch=1500, throttle=1000, yaw=1500,
               aux1=1000, aux2=1000, aux3=1000, aux4=1000):
    # Prepare 8 channels (each 16-bit little-endian = 2 bytes)
    channels = [roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4]
    payload = []
    for value in channels:
        payload.append(value & 0xFF)  # Low byte
        payload.append((value >> 8) & 0xFF)  # High byte
    send_msp_command(ser, MSP_SET_RAW_RC, payload)

def setup_serial_connection():
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        print("Connected to flight controller.")
        return ser
    except Exception as e:
        print("Error opening serial port:", e)
        exit(1)

def drone_mission():
    ser = setup_serial_connection()

    try:
        # 1. Arm the flight controller.
        print("Arming the drone...")
        set_arm(ser, True)
        time.sleep(3)  # Allow extra time for the FC to register arming

        # 2. Increase throttle to test motor spin.
        print("Increasing throttle to test motor spin...")
        set_raw_rc(ser, roll=1500, pitch=1500, throttle=1300, yaw=1500)  # Increase throttle
        time.sleep(5)  # Allow motors to run

        # 3. Return throttle to safe low value.
        print("Reducing throttle...")
        set_raw_rc(ser, roll=1500, pitch=1500, throttle=1000, yaw=1500)
        time.sleep(2)

        # 4. Disarm the flight controller.
        print("Disarming the drone...")
        set_arm(ser, False)
        time.sleep(2)

    except Exception as e:
        print("An error occurred during the mission:", e)
    finally:
        ser.close()
        print("Mission complete. Connection closed.")

if __name__ == "__main__":
    drone_mission()