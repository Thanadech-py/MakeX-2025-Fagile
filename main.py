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

left_forward_wheel = encoder_motor_class("M2", "INDEX1")
right_forward_wheel = encoder_motor_class("M3", "INDEX1")
left_back_wheel = encoder_motor_class("M5", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")

MAX_SPEED = 350
SPEED_MULTIPLIER = 2.1
PID_SPEED_MULTIPLIER = 2.1
BL_POWER = 100


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0, max_integral=1000, max_output=255):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value (target)
        self.integral = 0  # Sum of errors over time
        self.previous_error = 0  # Previous error (used for derivative)
        self.max_integral = max_integral  # Maximum integral value to prevent windup
        self.max_output = max_output  # Maximum output value
        self.previous_time = time.time()  # For delta time calculation
        self.ff_gain = 0.1  # Feedforward gain

    def update(self, current_value):
        # Calculate delta time
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        # Calculate the error (setpoint - current value)
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        # Clamp integral to prevent windup
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        I = self.Ki * self.integral

        # Derivative term with error clamping
        error_change = error - self.previous_error
        # Clamp error change to prevent spikes
        error_change = max(min(error_change, 100), -100)
        D = self.Kd * (error_change / dt) if dt > 0 else 0

        # Feedforward term
        FF = self.ff_gain * self.setpoint

        # Calculate the output
        output = P + I + D + FF

        # Clamp output to prevent motor damage
        output = max(min(output, self.max_output), -self.max_output)

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0  # Reset the integral to avoid wind-up
        self.previous_error = 0  # Reset previous error to avoid a large derivative spike
        self.previous_time = time.time()  # Reset time for delta time calculation


class motors:
    
    def drive(lf: int, lb: int, rf: int, rb: int):
        left_back_wheel.set_speed(lb) # left back :DDDDD
        right_back_wheel.set_speed(-rb)  # RIGHT BACK  
        right_forward_wheel.set_speed(-(rf))      # RIGHT FORWARD
        left_forward_wheel.set_speed(lf)             # LEFT BACK
    
    def stop():
        motors.drive(0, 0, 0, 0)
        
class util:
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)
        
    def apply_deadzone(value, deadzone):
        if abs(value) < deadzone:
            return 0
        return value
        
    def apply_response_curve(value, curve=2.0):
        # Apply a power curve to make the response more natural
        # curve > 1 makes the response more sensitive at low values
        # curve < 1 makes the response less sensitive at low values
        sign = 1 if value >= 0 else -1
        return sign * (abs(value) ** curve)
        
    def scale_joystick(value, max_speed=100):
        # Scale joystick value (-100 to 100) to motor speed (-max_speed to max_speed)
        return (value / 100.0) * max_speed

class MotionProfile:
    def __init__(self, max_velocity, max_acceleration, max_jerk):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk
        self.current_position = 0
        self.current_velocity = 0
        self.current_acceleration = 0
        self.target_position = 0
        self.last_time = time.time()

    def update(self, target_position):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Calculate position error
        position_error = target_position - self.current_position

        # Calculate desired velocity using trapezoidal profile
        if abs(position_error) < 0.1:  # Close enough to target
            desired_velocity = 0
        else:
            # Calculate maximum possible velocity at current position
            max_possible_velocity = math.sqrt(2 * self.max_acceleration * abs(position_error))
            desired_velocity = math.copysign(min(abs(position_error) / dt, max_possible_velocity, self.max_velocity), position_error)

        # Calculate velocity error
        velocity_error = desired_velocity - self.current_velocity

        # Update acceleration with jerk limit
        desired_acceleration = velocity_error / dt
        acceleration_change = desired_acceleration - self.current_acceleration
        acceleration_change = max(min(acceleration_change, self.max_jerk * dt), -self.max_jerk * dt)
        self.current_acceleration += acceleration_change

        # Update velocity with acceleration limit
        self.current_velocity += self.current_acceleration * dt
        self.current_velocity = max(min(self.current_velocity, self.max_velocity), -self.max_velocity)

        # Update position
        self.current_position += self.current_velocity * dt

        return self.current_position, self.current_velocity, self.current_acceleration

class AdvancedPID(PID):
    def __init__(self, Kp, Ki, Kd, setpoint=0, max_integral=1000, max_output=255):
        super().__init__(Kp, Ki, Kd, setpoint, max_integral, max_output)
        self.error_history = []
        self.max_history = 10
        self.adaptive_gains = {
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd
        }
        self.learning_rate = 0.001

    def update(self, current_value):
        # Calculate error
        error = self.setpoint - current_value
        
        # Update error history
        self.error_history.append(error)
        if len(self.error_history) > self.max_history:
            self.error_history.pop(0)
        
        # Calculate error statistics
        error_mean = sum(self.error_history) / len(self.error_history)
        error_variance = sum((e - error_mean) ** 2 for e in self.error_history) / len(self.error_history)
        
        # Adaptive gains based on error statistics
        if error_variance > 100:  # High oscillation
            self.adaptive_gains["Kp"] *= 0.95
            self.adaptive_gains["Ki"] *= 0.95
        elif error_variance < 10:  # Low oscillation
            self.adaptive_gains["Kp"] *= 1.05
            self.adaptive_gains["Ki"] *= 1.05
        
        # Update PID with adaptive gains
        P = self.adaptive_gains["Kp"] * error
        self.integral += error
        I = self.adaptive_gains["Ki"] * self.integral
        D = self.adaptive_gains["Kd"] * (error - self.previous_error)
        
        output = P + I + D
        self.previous_error = error
        
        return output

class holonomic:    
    pids = {
        "lf": AdvancedPID(Kp=1.2, Ki=0.1, Kd=0.05, max_integral=1000, max_output=255),
        "lb": AdvancedPID(Kp=1.2, Ki=0.1, Kd=0.05, max_integral=1000, max_output=255),
        "rf": AdvancedPID(Kp=1.1, Ki=0.1, Kd=0.05, max_integral=1000, max_output=255),
        "rb": AdvancedPID(Kp=1.1, Ki=0.1, Kd=0.05, max_integral=1000, max_output=255),
    }

    # Motor tuning for different movement types
    tune = {
        "slide": {
            "fl": 1.5,  # front left wheel during sliding
            "fr": 1.5,  # front right wheel during sliding
            "bl": 1.5,  # back left wheel during sliding
            "br": 1.5   # back right wheel during sliding
        },
        "forward": {
            "fl": 1.0,  # front left wheel during forward movement
            "fr": 1.0,  # front right wheel during forward movement
            "bl": 1.0,  # back left wheel during forward movement
            "br": 1.0   # back right wheel during forward movement
        },
        "turn": {
            "fl": 1.0,  # front left wheel during turning
            "fr": 1.0,  # front right wheel during turning
            "bl": 1.0,  # back left wheel during turning
            "br": 1.0   # back right wheel during turning
        }
    }

    # Auto-tuning parameters
    auto_tune = {
        "enabled": False,
        "last_error": {
            "slide": {"fl": 0, "fr": 0, "bl": 0, "br": 0},
            "forward": {"fl": 0, "fr": 0, "bl": 0, "br": 0},
            "turn": {"fl": 0, "fr": 0, "bl": 0, "br": 0}
        },
        "tune_step": 0.05,  # How much to adjust tuning values each step
        "min_tune": 0.5,    # Minimum tuning value
        "max_tune": 2.0     # Maximum tuning value
    }

    # Motion profiles for each movement type
    motion_profiles = {
        "slide": MotionProfile(max_velocity=100, max_acceleration=50, max_jerk=20),
        "forward": MotionProfile(max_velocity=100, max_acceleration=50, max_jerk=20),
        "turn": MotionProfile(max_velocity=100, max_acceleration=50, max_jerk=20)
    }

    def auto_tune_wheels(movement_type, target_speed, actual_speeds):
        if not holonomic.auto_tune["enabled"]:
            return
            
        # Calculate errors for each wheel
        errors = {
            "fl": target_speed - actual_speeds["fl"],
            "fr": target_speed - actual_speeds["fr"],
            "bl": target_speed - actual_speeds["bl"],
            "br": target_speed - actual_speeds["br"]
        }
        
        # Update tuning values based on errors
        for wheel in ["fl", "fr", "bl", "br"]:
            error = errors[wheel]
            last_error = holonomic.auto_tune["last_error"][movement_type][wheel]
            
            # If error is getting worse, adjust tuning in opposite direction
            if abs(error) > abs(last_error):
                if error > 0:
                    holonomic.tune[movement_type][wheel] -= holonomic.auto_tune["tune_step"]
                else:
                    holonomic.tune[movement_type][wheel] += holonomic.auto_tune["tune_step"]
            # If error is getting better, keep adjusting in same direction
            else:
                if error > 0:
                    holonomic.tune[movement_type][wheel] += holonomic.auto_tune["tune_step"]
                else:
                    holonomic.tune[movement_type][wheel] -= holonomic.auto_tune["tune_step"]
            
            # Clamp tuning values
            holonomic.tune[movement_type][wheel] = max(
                min(holonomic.tune[movement_type][wheel], holonomic.auto_tune["max_tune"]),
                holonomic.auto_tune["min_tune"]
            )
            
            # Update last error
            holonomic.auto_tune["last_error"][movement_type][wheel] = error

    def drive(vx, vy, wL, deadzone=25, pid=False):
        global SPEED_MULTIPLIER, PID_SPEED_MULTIPLIER
        
        # Apply motion profiling
        if math.fabs(vx) > math.fabs(vy) and vx > 0:
            # Sliding movement
            vx, _, _ = holonomic.motion_profiles["slide"].update(vx)
        elif math.fabs(vy) > math.fabs(vx) and vy > 0:
            # Forward/backward movement
            vy, _, _ = holonomic.motion_profiles["forward"].update(vy)
        elif math.fabs(wL) > math.fabs(vx) and math.fabs(wL) > math.fabs(vy):
            # Turning movement
            wL, _, _ = holonomic.motion_profiles["turn"].update(wL)

        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0

        # Ensure the correct speed multiplier
        multiplier = PID_SPEED_MULTIPLIER if pid else SPEED_MULTIPLIER
            
        # Calculation for the wheel speed with improved kinematics
        vFL = (vx + (vy * 1.2) + wL) * multiplier
        vFR = (-(vx) + (vy * 1.2) - wL) * multiplier
        vBL = (-(vx) + (vy * 1.2) + wL) * multiplier
        vBR = (vx + (vy * 1.2) - wL) * multiplier
        
        # Store target speeds for auto-tuning
        target_speeds = {"fl": vFL, "fr": vFR, "bl": vBL, "br": vBR}
        
        # Apply tuning based on movement type
        if math.fabs(vx) > math.fabs(vy) and vx > 0:
            # Sliding movement
            vFL *= holonomic.tune["slide"]["fl"]
            vFR *= holonomic.tune["slide"]["fr"]
            vBL *= holonomic.tune["slide"]["bl"]
            vBR *= holonomic.tune["slide"]["br"]
            
            # Auto-tune for sliding
            if pid:
                actual_speeds = {
                    "fl": -left_forward_wheel.get_value("speed"),
                    "fr": right_forward_wheel.get_value("speed"),
                    "bl": -left_back_wheel.get_value("speed"),
                    "br": right_back_wheel.get_value("speed")
                }
                holonomic.auto_tune_wheels("slide", vx, actual_speeds)
        elif math.fabs(vy) > math.fabs(vx) and vy > 0:
            # Forward/backward movement
            vFL *= holonomic.tune["forward"]["fl"]
            vFR *= holonomic.tune["forward"]["fr"]
            vBL *= holonomic.tune["forward"]["bl"]
            vBR *= holonomic.tune["forward"]["br"]
            
            # Auto-tune for forward/backward
            if pid:
                actual_speeds = {
                    "fl": -left_forward_wheel.get_value("speed"),
                    "fr": right_forward_wheel.get_value("speed"),
                    "bl": -left_back_wheel.get_value("speed"),
                    "br": right_back_wheel.get_value("speed")
                }
                holonomic.auto_tune_wheels("forward", vy, actual_speeds)
        elif math.fabs(wL) > math.fabs(vx) and math.fabs(wL) > math.fabs(vy):
            # Turning movement
            vFL *= holonomic.tune["turn"]["fl"]
            vFR *= holonomic.tune["turn"]["fr"]
            vBL *= holonomic.tune["turn"]["bl"]
            vBR *= holonomic.tune["turn"]["br"]
            
            # Auto-tune for turning
            if pid:
                actual_speeds = {
                    "fl": -left_forward_wheel.get_value("speed"),
                    "fr": right_forward_wheel.get_value("speed"),
                    "bl": -left_back_wheel.get_value("speed"),
                    "br": right_back_wheel.get_value("speed")
                }
                holonomic.auto_tune_wheels("turn", wL, actual_speeds)
        
        if pid:            
            # Left Forward
            holonomic.pids["lf"].set_setpoint(vFL)
            vFL = holonomic.pids["lf"].update(-left_forward_wheel.get_value("speed"))
            # Left Back
            holonomic.pids["lb"].set_setpoint(vBL)
            vBL = holonomic.pids["lb"].update(-left_back_wheel.get_value("speed"))
            # Right Forward
            holonomic.pids["rf"].set_setpoint(vFR)
            vFR = holonomic.pids["rf"].update(right_forward_wheel.get_value("speed"))
            # Right Back
            holonomic.pids["rb"].set_setpoint(vBR)
            vBR = holonomic.pids["rb"].update(right_back_wheel.get_value("speed"))

        # Velocity
        vFL = util.restrict(vFL, -MAX_SPEED, MAX_SPEED)
        vFR = util.restrict(vFR, -MAX_SPEED, MAX_SPEED)
        vBL = util.restrict(vBL, -MAX_SPEED, MAX_SPEED)
        vBR = util.restrict(vBR, -MAX_SPEED, MAX_SPEED)
        # Drive motor
        motors.drive(vFL, vBL, vFR, vBR)
        
    def move_forward(power):
        holonomic.drive(0, power, 0)
        
    def move_backward(power):
        holonomic.drive(0, -power, 0)
        
    def slide_right(power):
        holonomic.drive(power, 0, 0)
        
    def slide_left(power):
        holonomic.drive(-power, 0, 0)
        
    def turn_right(power):
        holonomic.drive(0, 0, power)
        
    def turn_left(power):
        holonomic.drive(0, 0, -power)

class Auto:
    def right():
        pass
    
    def left():
        pass

class dc_motor:
    # Default DC port
    dc_port = "DC1"
    # Default direction (not reversed)
    reverse = False
    
    # Initialize DC motor with a specific port
    def __init__(self, port: str) -> None:
        self.dc_port = port
        
    # Method to turn on the DC motor
    def on(self, Power: int, rev: bool = None) -> None:
        if rev is not None:
            self.reverse = rev
        power = -Power if self.reverse else Power
        power_expand_board.set_power(self.dc_port, power)
        
    # Method to turn off the DC motor
    def off(self) -> None:
        power_expand_board.stop(self.dc_port)
        
class brushless_motor:
    # Default brushless motor port
    bl_port = "BL1"
    
    # Initialize brushless motor with a specific port
    def __init__(self, port: str) -> None:
        self.bl_port = port
        
    # Method to turn on the brushless motor
    def on(self) -> None:
        power_expand_board.set_power(self.bl_port, BL_POWER)
        
    # Method to turn off the brushless motor
    def off(self) -> None:
        power_expand_board.stop(self.bl_port)

      

class runtime:
    # Define control mode
    CTRL_MODE = 0
    
    # Robot state
    ENABLED = True
    
    # Joystick settings
    JOYSTICK_DEADZONE = 15
    RESPONSE_CURVE = 1.5
    MAX_SPEED = 100
    
    # Joystick PID controllers
    joystick_pids = {
        "lx": PID(Kp=0.8, Ki=0.05, Kd=0.1, max_integral=500, max_output=100),
        "ly": PID(Kp=0.8, Ki=0.05, Kd=0.1, max_integral=500, max_output=100),
        "rx": PID(Kp=0.8, Ki=0.05, Kd=0.1, max_integral=500, max_output=100)
    }
    
    def process_joystick(value, axis):
        # Apply deadzone
        value = util.apply_deadzone(value, runtime.JOYSTICK_DEADZONE)
        if value == 0:
            runtime.joystick_pids[axis].set_setpoint(0)
            return 0
            
        # Apply response curve
        value = util.apply_response_curve(value, runtime.RESPONSE_CURVE)
        
        # Update PID controller
        runtime.joystick_pids[axis].set_setpoint(value)
        pid_output = runtime.joystick_pids[axis].update(value)
        
        # Scale to max speed
        return util.scale_joystick(pid_output, runtime.MAX_SPEED)
    
    def move_1():
        # Get and process joystick values with PID
        lx = runtime.process_joystick(gamepad.get_joystick("Lx"), "lx")
        ly = runtime.process_joystick(gamepad.get_joystick("Ly"), "ly")
        rx = runtime.process_joystick(gamepad.get_joystick("Rx"), "rx")
        
        # Only drive if any joystick is active
        if lx != 0 or ly != 0 or rx != 0:
            holonomic.drive(-lx, ly, -rx, pid=True)
        else:
            motors.drive(0, 0, 0, 0)

    def move_2():
        # Get and process joystick values with PID
        lx = runtime.process_joystick(gamepad.get_joystick("Lx"), "lx")
        ly = runtime.process_joystick(gamepad.get_joystick("Ly"), "ly")
        rx = runtime.process_joystick(gamepad.get_joystick("Rx"), "rx")
        
        # Only drive if any joystick is active
        if lx != 0 or ly != 0 or rx != 0:
            holonomic.drive(lx, -ly, -rx, pid=True)
        else:
            motors.drive(0, 0, 0, 0)
    def change_mode():
        if novapi.timer() > 0.9:
            if runtime.CTRL_MODE == 0:
                runtime.CTRL_MODE = 1
            else:
                runtime.CTRL_MODE = 0
            novapi.reset_timer()

class shoot_mode:
    # Method to control various robot functions based on button inputs
    def control_button():
        if gamepad.is_key_pressed("N2"):
            Vertical_intake.on(100, True)
            Shooter_feed.on(100, True)
            Front_intake.on(100, True)  
        elif gamepad.is_key_pressed("N3"):
            Vertical_intake.on(100, False)
            Shooter_feed.on(100, False)
            Front_intake.on(100, False)
        else:
            Vertical_intake.off()
            Shooter_feed.off()
            Front_intake.off()

        if gamepad.is_key_pressed("R1"):
            bl_1.on()
            bl_2.on()
        else:
            bl_1.off()
            bl_2.off()
            #shooter_angle control
        if gamepad.is_key_pressed("Up"):
            shooter.move(8, 10)

        elif gamepad.is_key_pressed("Down"):
            shooter.move(-8, 10)
        else:
            pass
 
class gripper_mode:
    # Method to control various robot functions based on button inputs
    def control_button():
        if gamepad.is_key_pressed("Up"):
            lift.on(100, True)
        elif gamepad.is_key_pressed("Down"):
            lift.on(100, False)
        else:
            lift.off()
        if gamepad.is_key_pressed("N1"):
            # gripper1.on(100)
            pass

        elif gamepad.is_key_pressed("N4"):
            # gripper1.on(100)
            pass
        else:
            # gripper1.off()
            pass
        

# Instantiate DC motors
lift = dc_motor("DC4") # using encoder for spacific position of lift functions
Shooter_feed = dc_motor("DC3")
Vertical_intake = dc_motor("DC2")
Front_intake = dc_motor("DC1")
bl_1 = brushless_motor("BL1")
bl_2 = brushless_motor("BL2")
shooter = smartservo_class("M1", "INDEX1") # only for angles


while True:
    if power_manage_module.is_auto_mode():
        pass
    else:
        runtime.move_1() 
        if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            runtime.change_mode()
        else:
            if runtime.CTRL_MODE == 0:
                shoot_mode.control_button()
            else:
                gripper_mode.control_button()