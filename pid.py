class PIDController:
    def __init__(self, Kp=1.0, Ki=0.1, Kd=0.01, setpoint=0, output_limits=(None, None)):
        """
        PID Controller implementation with anti-windup protection
        
        Args:
            Kp: Proportional gain (how aggressively to react to current error)
            Ki: Integral gain (how aggressively to react to accumulated error)
            Kd: Derivative gain (how aggressively to react to rate of error change)
            setpoint: Target value to control toward
            output_limits: Tuple of (min, max) values to clamp the output
        """
        self.Kp = Kp  # Proportional coefficient
        self.Ki = Ki  # Integral coefficient
        self.Kd = Kd  # Derivative coefficient
        self.setpoint = setpoint  # Target value
        self.output_limits = output_limits  # Output clamping limits
        
        # Internal state variables
        self._integral = 0  # Accumulated error
        self._prev_error = 0  # Previous error value
        self._last_time = None  # Last update timestamp

    def update(self, current_value, dt):
        """
        Calculate PID output based on current value and time delta
        
        Args:
            current_value: Current process value
            dt: Time step since last update
            
        Returns:
            Control output value
        """
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with conditional anti-windup
        if abs(error) < 0.1:  # Only accumulate error when close to setpoint
            self._integral += error * dt
        else:
            self._integral = 0  # Reset integral term when far from setpoint

        I = self.Ki * self._integral
        
        # Derivative term
        derivative = (error - self._prev_error) / dt if dt > 0 else 0
        D = self.Kd * derivative
        
        # Calculate total output
        output = P + I + D
        
        # Apply output limits
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)
        
        # Store error for next derivative calculation
        self._prev_error = error
        
        return output