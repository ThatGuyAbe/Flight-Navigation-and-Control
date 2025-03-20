class StabilizationThread(threading.Thread):
    def __init__(self, imu, depth_sensor_manager, esc_control_callback,
                 command_handler, user_tracker):
        super().__init__()
        self.daemon = True
        self.running = True
        
        # Hardware interfaces
        self.imu = imu
        self.depth_manager = depth_sensor_manager
        self.esc_control = esc_control_callback
        
        # WebSocket components
        self.command_handler = command_handler
        self.user_tracker = user_tracker
        
        # Flight states
        self.armed = False
        self.active_command = None
        self.base_throttle = 1500
        self.hover_throttle = 1550
        
        # PID Controllers
        self.pid_roll = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.pid_pitch = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.pid_yaw = PIDController(kp=0.5, ki=0.0, kd=0.05)
        
        # Control parameters
        self.sample_rate = 100  # Hz
        self.follow_gains = {'x': 0.1, 'y': 0.08, 'z': 0.05}

    def run(self):
        while self.running:
            start_time = time.time()
            
            # Process inputs and states
            self._process_commands()
            current_command = self._get_active_command()
            
            # Get sensor data
            yaw, pitch, roll = self.imu.get_angles()
            distances = self.depth_manager.distances
            
            # Core stabilization
            roll_corr = self.pid_roll.compute(roll)
            pitch_corr = self.pid_pitch.compute(pitch)
            yaw_corr = self.pid_yaw.compute(yaw)
            
            # Obstacle avoidance
            obstacle_corr = self._avoid_obstacles(distances)
            
            # Command execution or tracking
            if current_command:
                command_corr = self._execute_command(current_command)
                tracking_corr = {}
            else:
                command_corr = {}
                tracking_corr = self._track_user()
            
            # Combine corrections
            corrections = self._combine_corrections(
                roll_corr, pitch_corr, yaw_corr,
                obstacle_corr, command_corr, tracking_corr
            )
            
            # Apply motor controls
            self.esc_control(**corrections)
            
            # Maintain sample rate
            elapsed = time.time() - start_time
            time.sleep(max(0, (1.0 / self.sample_rate) - elapsed))

    def _process_commands(self):
        cmd = self.command_handler.get_command()
        if cmd:
            if cmd == 'takeoff':
                self._arm()
            elif cmd == 'land':
                self._land()
            elif cmd == 'return_to_base':
                self.active_command = 'return_to_base'

    def _get_active_command(self):
        if self.active_command == 'return_to_base' and self._near_base():
            self.active_command = None
        return self.active_command

    def _execute_command(self, command):
        if command == 'return_to_base':
            return self._return_to_base_corrections()
        return {}

    def _track_user(self):
        pos = self.user_tracker.get_position()
        if not pos or pos['z'] <= 0:
            return {}
            
        x_error = pos['x'] - self.user_tracker.frame_center[0]
        z_error = pos['z'] - 2000  # Target 2m distance
        y_error = pos['y'] - self.user_tracker.frame_center[1]
        
        return {
            'roll': -x_error * self.follow_gains['x'],
            'pitch': z_error * self.follow_gains['z'],
            'throttle': y_error * self.follow_gains['y']
        }

    def _combine_corrections(self, roll, pitch, yaw, obstacle, command, tracking):
        """Combine corrections with priority: stability > obstacles > commands > tracking"""
        combined = {
            'roll': roll + obstacle.get('roll', 0),
            'pitch': pitch + obstacle.get('pitch', 0),
            'yaw': yaw + obstacle.get('yaw', 0),
            'base_throttle': self.base_throttle
        }
        
        # Add command corrections
        combined['roll'] += command.get('roll', 0)
        combined['pitch'] += command.get('pitch', 0)
        combined['yaw'] += command.get('yaw', 0)
        
        # Add tracking corrections
        combined['roll'] += tracking.get('roll', 0)
        combined['pitch'] += tracking.get('pitch', 0)
        combined['base_throttle'] += tracking.get('throttle', 0)
        
        return combined

    def _arm(self):
        if not self.armed:
            self.armed = True
            self.base_throttle = self.hover_throttle
            set_arm(ser, True)

    def _land(self):
        self.active_command = None
        while self.base_throttle > 1000 and self.armed:
            self.base_throttle -= 10
            time.sleep(0.1)
        set_arm(ser, False)
        self.armed = False

    def _return_to_base_corrections(self):
        """Simplified return logic - implement proper navigation here"""
        return {'pitch': -50}  # Move backward

    def _near_base(self):
        """Implement actual position checking logic"""
        return False

    def _avoid_obstacles(self, distances):
        """Existing indoor obstacle avoidance implementation"""
        # [Previous implementation here]
        return {}

    def stop(self):
        self.running = False
        self.command_handler.stop()
        self.user_tracker.stop()
