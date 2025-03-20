# Main function
import threading
import time
import parallel
import stable
import RC_fly.py
import tc

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0, sample_time=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.output = 0
        self.lock = threading.Lock()

    def compute(self, input_value):
        with self.lock:
            now = time.time()
            dt = now - self.last_time
            
            if dt >= self.sample_time:
                error = self.setpoint - input_value
                self.integral += error * dt
                derivative = (error - self.last_error) / dt
                
                self.output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
                self.last_error = error
                self.last_time = now
                
            return self.output

    def update_coefficients(self, kp, ki, kd):
        with self.lock:
            self.kp = kp
            self.ki = ki
            self.kd = kd


def motor_mixer(roll, pitch, yaw, base_throttle=1500):
    """Convert PID outputs to motor values (X configuration)"""
    # Motor mixing matrix
    m1 = base_throttle + roll + pitch + yaw
    m2 = base_throttle - roll + pitch - yaw
    m3 = base_throttle - roll - pitch + yaw
    m4 = base_throttle + roll - pitch - yaw
    
    # Constrain motor values between 1000-2000 μs
    return [
        max(1000, min(2000, int(m1)),
        max(1000, min(2000, int(m2)),
        max(1000, min(2000, int(m3)),
        max(1000, min(2000, int(m4))
    ]

# === Main Program Integration ===
if __name__ == "__main__":
    # Hardware initialization
    imu = IMUThread()
    depth_mgr = SensorManager()
    ser = setup_serial_connection()
    
    # WebSocket components
    command_handler = CommandHandler()
    user_tracker = UserTracker()
    
    # Stabilization system
    def esc_control(roll, pitch, yaw, base_throttle):
        motors = motor_mixer(roll, pitch, yaw, base_throttle)
        set_motor_values(ser, motors)
    
    stabilizer = StabilizationThread(imu, depth_mgr, esc_control,
                                    command_handler, user_tracker)
    
    try:
        # Start all components
        imu.start()
        depth_mgr.start()
        command_handler.start()
        user_tracker.start()
        stabilizer.start()
        
        # Main loop
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        stabilizer.stop()
        set_arm(ser, False)
        ser.close()
        imu.stop()
        depth_mgr.stop()
        print("System shutdown complete")
