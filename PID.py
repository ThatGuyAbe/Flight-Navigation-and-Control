import threading
import time

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
