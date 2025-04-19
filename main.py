# Import necessary modules
import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
# from mbuild.led_matrix import led_matrix_class
# from mbuild.smart_camera import smart_camera_class
# from mbuild.ranging_sensor import ranging_sensor_class
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module
import time

# Motor constants
MAX_SPEED = 350
SPEED_MULTIPLIER = 2.1
PID_SPEED_MULTIPLIER = 2.1
BL_POWER = 100

# Initialize motors
left_forward_wheel = encoder_motor_class("M2", "INDEX1")
right_forward_wheel = encoder_motor_class("M3", "INDEX1")
left_back_wheel = encoder_motor_class("M5", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")


class PID:
    """PID controller class for motor control."""
    
    def __init__(self, kp, ki, kd, setpoint=0, max_integral=1000, max_output=255):
        """Initialize PID controller with gains and limits.
        
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain
            kd (float): Derivative gain
            setpoint (float): Target value
            max_integral (float): Maximum integral value
            max_output (float): Maximum output value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.max_integral = max_integral
        self.max_output = max_output
        self.previous_time = time.time()
        self.ff_gain = 0.1

    def update(self, current_value):
        """Update PID controller with new measurement.
        
        Args:
            current_value (float): Current measured value
            
        Returns:
            float: Control output
        """
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        error = self.setpoint - current_value
        
        # Proportional term
        p = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        i = self.ki * self.integral

        # Derivative term with error clamping
        error_change = error - self.previous_error
        error_change = max(min(error_change, 100), -100)
        d = self.kd * (error_change / dt) if dt > 0 else 0

        # Feedforward term
        ff = self.ff_gain * self.setpoint

        # Calculate output
        output = p + i + d + ff
        output = max(min(output, self.max_output), -self.max_output)

        self.previous_error = error
        return output

    def set_setpoint(self, setpoint):
        """Set new target value and reset controller state.
        
        Args:
            setpoint (float): New target value
        """
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.previous_time = time.time()


class Motors:
    """Class for controlling all robot motors."""
    
    @staticmethod
    def drive(lf, lb, rf, rb):
        """Drive all motors with specified speeds.
        
        Args:
            lf (int): Left forward speed
            lb (int): Left back speed
            rf (int): Right forward speed
            rb (int): Right back speed
        """
        left_back_wheel.set_speed(lb)
        right_back_wheel.set_speed(-rb)
        right_forward_wheel.set_speed(-rf)
        left_forward_wheel.set_speed(lf)
    
    @staticmethod
    def stop():
        """Stop all motors."""
        Motors.drive(0, 0, 0, 0)


class Util:
    """Utility class for common functions."""
    
    @staticmethod
    def restrict(val, minimum, maximum):
        """Restrict value to range.
        
        Args:
            val (float): Value to restrict
            minimum (float): Minimum allowed value
            maximum (float): Maximum allowed value
            
        Returns:
            float: Restricted value
        """
        return max(min(val, maximum), minimum)
        
    @staticmethod
    def apply_deadzone(value, deadzone):
        """Apply deadzone to value.
        
        Args:
            value (float): Input value
            deadzone (float): Deadzone threshold
            
        Returns:
            float: Value with deadzone applied
        """
        if abs(value) < deadzone:
            return 0
        return value
        
    @staticmethod
    def apply_response_curve(value, curve=2.0):
        """Apply response curve to value.
        
        Args:
            value (float): Input value
            curve (float): Curve exponent
            
        Returns:
            float: Value with response curve applied
        """
        sign = 1 if value >= 0 else -1
        return sign * (abs(value) ** curve)
        
    @staticmethod
    def scale_joystick(value, max_speed=100):
        """Scale joystick value to motor speed.
        
        Args:
            value (float): Joystick value (-100 to 100)
            max_speed (float): Maximum motor speed
            
        Returns:
            float: Scaled motor speed
        """
        return (value / 100.0) * max_speed


class DCMotor:
    """DC motor control class."""
    
    def __init__(self, port):
        """Initialize DC motor with port.
        
        Args:
            port (str): Motor port (e.g., "DC1")
        """
        self.port = port
        self.reverse = False
        
    def on(self, power, reverse=None):
        """Turn on motor with specified power and direction.
        
        Args:
            power (int): Motor power (0-100)
            reverse (bool, optional): Motor direction
        """
        if reverse is not None:
            self.reverse = reverse
        power = -power if self.reverse else power
        power_expand_board.set_power(self.port, power)
        
    def off(self):
        """Turn off motor."""
        power_expand_board.stop(self.port)


class BrushlessMotor:
    """Brushless motor control class."""
    
    def __init__(self, port):
        """Initialize brushless motor with port.
        
        Args:
            port (str): Motor port (e.g., "BL1")
        """
        self.port = port
        
    def on(self):
        """Turn on brushless motor."""
        power_expand_board.set_power(self.port, BL_POWER)
        
    def off(self):
        """Turn off brushless motor."""
        power_expand_board.stop(self.port)


# Instantiate motors
lift = DCMotor("DC5")
shooter_feed = DCMotor("DC3")
vertical_intake = DCMotor("DC2")
front_intake = DCMotor("DC1")
bl_1 = BrushlessMotor("BL1")
bl_2 = BrushlessMotor("BL2")
shooter = smartservo_class("M1", "INDEX1")


class Holonomic:
    """Holonomic drive control class."""
    
    # PID controllers for each wheel
    pids = {
        "lf": PID(kp=1.2, ki=0.1, kd=0.05, max_integral=1000, max_output=255),
        "lb": PID(kp=1.2, ki=0.1, kd=0.05, max_integral=1000, max_output=255),
        "rf": PID(kp=1.1, ki=0.1, kd=0.05, max_integral=1000, max_output=255),
        "rb": PID(kp=1.1, ki=0.1, kd=0.05, max_integral=1000, max_output=255),
    }

    @staticmethod
    def drive(vx, vy, wl, deadzone=25, pid=False):
        """Drive the robot using holonomic kinematics.
        
        Args:
            vx (float): X velocity (sideways)
            vy (float): Y velocity (forward/backward)
            wl (float): Angular velocity (rotation)
            deadzone (float): Deadzone threshold for joystick inputs
            pid (bool): Whether to use PID control
        """
        # Apply deadzone
        if abs(vx) < deadzone:
            vx = 0
        if abs(vy) < deadzone:
            vy = 0
        if abs(wl) < deadzone:
            wl = 0

        # Calculate wheel speeds using holonomic kinematics
        multiplier = PID_SPEED_MULTIPLIER if pid else SPEED_MULTIPLIER
        
        # Front Left
        vfl = (vx + vy + wl) * multiplier
        # Front Right
        vfr = (-vx + vy - wl) * multiplier
        # Back Left
        vbl = (-vx + vy + wl) * multiplier
        # Back Right
        vbr = (vx + vy - wl) * multiplier

        if pid:
            # Update PID controllers and get corrected speeds
            Holonomic.pids["lf"].set_setpoint(vfl)
            vfl = Holonomic.pids["lf"].update(-left_forward_wheel.get_value("speed"))
            
            Holonomic.pids["lb"].set_setpoint(vbl)
            vbl = Holonomic.pids["lb"].update(-left_back_wheel.get_value("speed"))
            
            Holonomic.pids["rf"].set_setpoint(vfr)
            vfr = Holonomic.pids["rf"].update(right_forward_wheel.get_value("speed"))
            
            Holonomic.pids["rb"].set_setpoint(vbr)
            vbr = Holonomic.pids["rb"].update(right_back_wheel.get_value("speed"))

        # Restrict speeds to maximum
        vfl = Util.restrict(vfl, -MAX_SPEED, MAX_SPEED)
        vfr = Util.restrict(vfr, -MAX_SPEED, MAX_SPEED)
        vbl = Util.restrict(vbl, -MAX_SPEED, MAX_SPEED)
        vbr = Util.restrict(vbr, -MAX_SPEED, MAX_SPEED)

        # Drive motors
        Motors.drive(vfl, vbl, vfr, vbr)

    @staticmethod
    def move_forward(power):
        """Move robot forward.
        
        Args:
            power (float): Power level (0-100)
        """
        Holonomic.drive(0, power, 0)
        
    @staticmethod
    def move_backward(power):
        """Move robot backward.
        
        Args:
            power (float): Power level (0-100)
        """
        Holonomic.drive(0, -power, 0)
        
    @staticmethod
    def slide_right(power):
        """Slide robot right.
        
        Args:
            power (float): Power level (0-100)
        """
        Holonomic.drive(power, 0, 0)
        
    @staticmethod
    def slide_left(power):
        """Slide robot left.
        
        Args:
            power (float): Power level (0-100)
        """
        Holonomic.drive(-power, 0, 0)
        
    @staticmethod
    def turn_right(power):
        """Turn robot right.
        
        Args:
            power (float): Power level (0-100)
        """
        Holonomic.drive(0, 0, power)
        
    @staticmethod
    def turn_left(power):
        """Turn robot left.
        
        Args:
            power (float): Power level (0-100)
        """
        Holonomic.drive(0, 0, -power)


class Runtime:
    """Runtime control class."""
    
    # Control mode
    CTRL_MODE = 0
    ENABLED = True
    
    # Joystick settings
    JOYSTICK_DEADZONE = 15
    RESPONSE_CURVE = 1.5
    MAX_SPEED = 100
    
    # PID controllers for joystick
    joystick_pids = {
        "lx": PID(kp=0.8, ki=0.05, kd=0.1, max_integral=500, max_output=100),
        "ly": PID(kp=0.8, ki=0.05, kd=0.1, max_integral=500, max_output=100),
        "rx": PID(kp=0.8, ki=0.05, kd=0.1, max_integral=500, max_output=100)
    }
    
    @staticmethod
    def process_joystick(value, axis):
        """Process joystick input with PID control.
        
        Args:
            value (float): Joystick value
            axis (str): Axis identifier
            
        Returns:
            float: Processed value
        """
        value = Util.apply_deadzone(value, Runtime.JOYSTICK_DEADZONE)
        if value == 0:
            Runtime.joystick_pids[axis].set_setpoint(0)
            return 0
            
        value = Util.apply_response_curve(value, Runtime.RESPONSE_CURVE)
        Runtime.joystick_pids[axis].set_setpoint(value)
        pid_output = Runtime.joystick_pids[axis].update(value)
        return Util.scale_joystick(pid_output, Runtime.MAX_SPEED)
    
    @staticmethod
    def move_1():
        """Control robot movement in mode 1."""
        lx = Runtime.process_joystick(gamepad.get_joystick("Lx"), "lx")
        ly = Runtime.process_joystick(gamepad.get_joystick("Ly"), "ly")
        rx = Runtime.process_joystick(gamepad.get_joystick("Rx"), "rx")
        
        if lx != 0 or ly != 0 or rx != 0:
            Holonomic.drive(-lx, ly, -rx, pid=True)
        else:
            Motors.stop()

    @staticmethod
    def move_2():
        """Control robot movement in mode 2."""
        lx = Runtime.process_joystick(gamepad.get_joystick("Lx"), "lx")
        ly = Runtime.process_joystick(gamepad.get_joystick("Ly"), "ly")
        rx = Runtime.process_joystick(gamepad.get_joystick("Rx"), "rx")
        
        if lx != 0 or ly != 0 or rx != 0:
            Holonomic.drive(lx, -ly, -rx, pid=True)
        else:
            Motors.stop()

    @staticmethod
    def change_mode():
        """Switch between control modes."""
        if novapi.timer() > 0.9:
            Runtime.CTRL_MODE = 1 if Runtime.CTRL_MODE == 0 else 0
            novapi.reset_timer()


class ShootMode:
    """Shooting mode control class."""
    
    @staticmethod
    def control_button():
        """Control shooting mode functions."""
        if gamepad.is_key_pressed("N2"):
            vertical_intake.on(100, True)
            shooter_feed.on(100, True)
            front_intake.on(100, True)
        elif gamepad.is_key_pressed("N3"):
            vertical_intake.on(100, False)
            shooter_feed.on(100, False)
            front_intake.on(100, False)
        else:
            vertical_intake.off()
            shooter_feed.off()
            front_intake.off()

        if gamepad.is_key_pressed("R1"):
            bl_1.on()
            bl_2.on()
        else:
            bl_1.off()
            bl_2.off()

        if gamepad.is_key_pressed("Up"):
            shooter.move(8, 10)
        elif gamepad.is_key_pressed("Down"):
            shooter.move(-8, 10)


class GripperMode:
    """Gripper mode control class."""
    
    @staticmethod
    def control_button():
        """Control gripper mode functions."""
        if gamepad.is_key_pressed("Up"):
            lift.on(100, True)
        elif gamepad.is_key_pressed("Down"):
            lift.on(100, False)
        else:
            lift.off()


# Main loop
while True:
    if power_manage_module.is_auto_mode():
        pass
    else:
        if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            Runtime.change_mode()
        else:
            if Runtime.CTRL_MODE == 0:
                ShootMode.control_button()
                Runtime.move_1()
            else:
                GripperMode.control_button()
                Runtime.move_2()