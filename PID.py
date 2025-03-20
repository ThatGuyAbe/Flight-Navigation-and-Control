# pid_controller.py
import threading
import time
import parallel

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

class StabilizationThread(threading.Thread):
    def __init__(self, imu, depth_sensor_manager, esc_control_callback):
        super().__init__()
        self.daemon = True
        self.running = True
        
        # Sensor interfaces
        self.imu = imu
        self.depth_manager = depth_sensor_manager
        
        # PID Controllers
        self.pid_roll = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.pid_pitch = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.pid_yaw = PIDController(kp=0.5, ki=0.0, kd=0.05)
        
        # Motor control callback
        self.esc_control = esc_control_callback
        self.sample_rate = 100  # Hz

    def run(self):
        while self.running:
            start_time = time.time()
            
            # Get latest sensor data
            yaw, pitch, roll = self.imu.get_angles()
            distances = self.depth_manager.distances  # Front, left, right distances
            
            # Compute PID corrections
            roll_corr = self.pid_roll.compute(roll)
            pitch_corr = self.pid_pitch.compute(pitch)
            yaw_corr = self.pid_yaw.compute(yaw)
            
            # Obstacle avoidance (priority 2)
            obstacle_corr = self._avoid_obstacles(distances)
            
            # Combine corrections
            final_roll = roll_corr + obstacle_corr['roll']
            final_pitch = pitch_corr + obstacle_corr['pitch']
            final_yaw = yaw_corr + obstacle_corr['yaw']
            
            # Send to motor control
            self.esc_control(final_roll, final_pitch, final_yaw)
            
            # Maintain sample rate
            elapsed = time.time() - start_time
            sleep_time = (1.0 / self.sample_rate) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _avoid_obstacles(self, distances):
        """Generate avoidance corrections for 5-directional sensing"""
        correction = {'roll': 0, 'pitch': 0, 'yaw': 0}
        safe_distance = 300  # mm
        avoidance_strength = 75  # Base correction value
        max_correction = 150  # Maximum allowed correction
    
        # Sensor Index Map (based on your initialization order)
        # 0: bottom, 1: right, 2: left, 3: back, 4: top
        
        # Bottom Protection (Emergency Ascend)
        if distances[0] < safe_distance:
            correction['pitch'] += min(
                avoidance_strength * (safe_distance - distances[0]) // 100,
                max_correction
            )
        
        # Top Protection (Emergency Descend)
        if distances[4] < safe_distance * 1.2:  # More sensitive for overhead obstacles
            correction['pitch'] -= min(
                avoidance_strength * (safe_distance - distances[4]) // 100,
                max_correction
            )
        
        # Horizontal Plane Obstacle Avoidance
        # Right Obstacle (Push left)
        if distances[1] < safe_distance:
            correction['roll'] -= min(
                avoidance_strength * (safe_distance - distances[1]) // 50,
                max_correction
            )
        
        # Left Obstacle (Push right)
        if distances[2] < safe_distance:
            correction['roll'] += min(
                avoidance_strength * (safe_distance - distances[2]) // 50,
                max_correction
            )
        
        # Rear Protection (Push forward)
        if distances[3] < safe_distance * 1.5:  # Larger buffer for rear obstacles
            correction['pitch'] += min(
                avoidance_strength * (safe_distance - distances[3]) // 75,
                max_correction
            )
    
        # Yaw correction for directional preference
        # Favor turning away from side obstacles
        if distances[1] < distances[2]:  # More obstacles on right
            correction['yaw'] = -avoidance_strength // 2
        elif distances[2] < distances[1]:  # More obstacles on left
            correction['yaw'] = avoidance_strength // 2
    
        # Limit maximum corrections
        for axis in ['roll', 'pitch', 'yaw']:
            correction[axis] = max(-max_correction, min(max_correction, correction[axis]))
    
        return correction

    def stop(self):
        self.running = False

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
    from your_esc_code import setup_serial_connection, set_arm, set_motor_values
    
    # Initialize hardware
    imu = IMUThread()
    depth_mgr = SensorManager()
    ser = setup_serial_connection()
    
    # Define ESC control callback
    def esc_control(roll, pitch, yaw):
        motors = motor_mixer(roll, pitch, yaw)
        set_motor_values(ser, motors)
    
    # Create stabilization system
    stabilizer = StabilizationThread(imu, depth_mgr, esc_control)
    
    try:
        # Start subsystems
        imu.start()
        depth_mgr.start()
        stabilizer.start()
        
        # Arm motors
        set_arm(ser, True)
        time.sleep(1)
        
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        stabilizer.stop()
        set_arm(ser, False)
        ser.close()
        imu.stop()
        depth_mgr.stop()
        print("System shutdown complete")
