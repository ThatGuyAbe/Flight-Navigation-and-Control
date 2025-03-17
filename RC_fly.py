import time
import serial

# Replace with the correct serial port (e.g., /dev/ttyACM0 or /dev/ttyUSB0)
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# MSP Command IDs
MSP_SET_ARM = 217  # Command to arm/disarm
MSP_SET_MOTOR = 214  # Command to set motor values

# Function to calculate checksum
def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

# Function to send MSP command
def send_msp_command(ser, command_id, data=[]):
    # MSP message format: $M <direction> <payload_length> <command_id> <data> <checksum>
    direction = ord('<')  # Send to flight controller
    payload_length = len(data)
    message = [ord('$'), ord('M'), direction, payload_length, command_id] + data
    checksum = calculate_checksum(message[3:])  # Checksum starts from direction byte
    message.append(checksum)
    ser.write(bytearray(message))
    time.sleep(0.1)  # Wait for response

# Function to arm/disarm the flight controller
def set_arm(ser, arm):
    # Arm command payload: 1 to arm, 0 to disarm
    send_msp_command(ser, MSP_SET_ARM, [1 if arm else 0])

# Function to set motor values
def set_motor_values(ser, motor_values):
    # Motor values must be in the range 1000-2000 (Betaflight standard)
    # Each value is split into two bytes (low and high)
    payload = []
    for value in motor_values:
        payload.append(value & 0xFF)  # Low byte
        payload.append((value >> 8) & 0xFF)  # High byte
    send_msp_command(ser, MSP_SET_MOTOR, payload)

# Open serial connection to Betaflight flight controller
def setup_serial_connection():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        return ser
    except Exception as e:
        print(f"Error opening serial port: {e}")
        exit(1)

# Main script logic
def main():
    # Setup serial connection to Betaflight
    ser = setup_serial_connection()

    try:
        # Arm the flight controller
        print("Arming the flight controller...")
        set_arm(ser, True)
        time.sleep(1)  # Wait for arming to take effect

        # Set motor values (e.g., 1000-2000 range, 1000 = off, 2000 = full throttle)
        print("Setting motor values...")
        motor_values = [1500, 1500, 1500, 1500]  # Set all motors to 1500 (mid-throttle)
        set_motor_values(ser, motor_values)
        time.sleep(3)  # Motors run for 3 seconds

        # Stop motors
        print("Stopping motors...")
        motor_values = [1000, 1000, 1000, 1000]  # Set all motors to 1000 (off)
        set_motor_values(ser, motor_values)

        # Disarm the flight controller
        print("Disarming the flight controller...")
        set_arm(ser, False)

    finally:
        # Close serial connection
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
