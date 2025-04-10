# Import necessary modules
import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.led_matrix import led_matrix_class
from mbuild.smart_camera import smart_camera_class
from mbuild.ranging_sensor import ranging_sensor_class
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module
import time

left_forward_wheel = encoder_motor_class("M2", "INDEX1")
right_forward_wheel = encoder_motor_class("M3", "INDEX1")
left_back_wheel = encoder_motor_class("M5", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")

MAX_SPEED = 255
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

class holonomic:    

    pids = {
        "lf": PID(Kp=1.2, Ki=0.1, Kd=0.05, max_integral=1000, max_output=255),
        "lb": PID(Kp=1.2, Ki=0.1, Kd=0.05, max_integral=1000, max_output=255),
        "rf": PID(Kp=1.1, Ki=0.1, Kd=0.05, max_integral=1000, max_output=255),
        "rb": PID(Kp=1.1, Ki=0.1, Kd=0.05, max_integral=1000, max_output=255),
    }

    # motor tune
    tune = {
        "fl": 1.5,
        "fr": 1.5,
        "bl": 1.5,
        "br": 1.5,
    }

    def drive(vx, vy, wL, deadzone=25, pid=False):
        global SPEED_MULTIPLIER, PID_SPEED_MULTIPLIER
        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0

        # Ensure the correct speed multiplier
        multiplier = PID_SPEED_MULTIPLIER if pid else SPEED_MULTIPLIER
            
        # Calculation for the wheel speed
        vFL = (vx + (vy * 1.2) + wL) * multiplier
        vFR = (-(vx) + (vy * 1.2) - wL) * multiplier
        vBL = (-(vx) + (vy * 1.2) + wL) * multiplier
        vBR = (vx + (vy * 1.2) - wL) * multiplier
        
        # Sliding check to not interfere with the normal movement, incase of tuning specific power
        if math.fabs(vx) > math.fabs(vy) and vx > 0:
            vFL *= holonomic.tune["fl"] # หน้าซ้าย
            vFL *= holonomic.tune["fr"] # หน้าขวา
            vBL *= holonomic.tune["bl"] # หลังซ้าย
            vBR *= holonomic.tune["br"] # หลังขวา
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
        entrance_feed.set_reverse(True)
        entrance_feed.on(100)
        feeder.set_reverse(False)
        feeder.on(25)
        front_input.set_reverse(True)
        front_input.on(60)
        holonomic.move_forward(50) #move forward 
        time.sleep(2.51)
        motors.stop()
        holonomic.turn_left(60) #turn around
        time.sleep(1.93)
        motors.stop()
        holonomic.slide_left(45) #slide for discs
        time.sleep(2)
        motors.stop()
        holonomic.move_forward(25) #move to take discs
        time.sleep(3.6)
        motors.stop()
        holonomic.slide_left(45)
        time.sleep(1.2)
        motors.stop()
        holonomic.turn_right(60)
        time.sleep(1.92)
        motors.stop()
        holonomic.move_forward(25)
        time.sleep(2)
        motors.stop()
        time.sleep(500000)

class dc_motor:
    # Default DC port
    dc_port = "DC1"
    # Default direction (not reversed)
    reverse = False
    
    # Initialize DC motor with a specific port
    def __init__(self, port: str) -> None:
        self.dc_port = port
        
    # Method to set the direction of the motor
    def set_reverse(self, rev: bool) -> None:
        self.reverse = rev
        
    # Method to turn on the DC motor
    def on(self, Power: int) -> None:
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
            entrance_feed.off()
            feeder.off()
            conveyer.off()
            front_input.off()
            if runtime.CTRL_MODE == 0:
                runtime.CTRL_MODE = 1
            else:
                runtime.CTRL_MODE = 0
            novapi.reset_timer()

class shoot_mode:
    # Method to control various robot functions based on button inputs
    def control_button():
        if gamepad.is_key_pressed("N2"):
            entrance_feed.set_reverse(True)
            entrance_feed.on(100)
            feeder.set_reverse(False)
            feeder.on(100)
            conveyer.set_reverse(True)
            conveyer.on(100)
            front_input.set_reverse(True)
            front_input.on(100)
        elif gamepad.is_key_pressed("N3"):
            entrance_feed.set_reverse(False)
            entrance_feed.on(100)
            feeder.set_reverse(True)
            feeder.on(100)
            conveyer.set_reverse(False)
            conveyer.on(100)
            front_input.set_reverse(False)
            front_input.on(100)
        else:
            entrance_feed.off()
            feeder.off()
            conveyer.off()
            front_input.off()
        if gamepad.is_key_pressed("R1"):
            bl_1.on()
            bl_2.on()
        else:
            bl_1.off()
            bl_2.off()
        if gamepad.is_key_pressed("≡"):
            laser.set_reverse(True)
            laser.on(100)
        elif gamepad.is_key_pressed("+"):
            laser.off()
        else:
            pass
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
            lift.set_power(-90)
        elif gamepad.is_key_pressed("Down"):
            lift.set_power(90)
        else:
            lift.set_speed(0)
        if gamepad.is_key_pressed("N1"):
            gripper1.set_reverse(True)
            gripper1.on(100)

        elif gamepad.is_key_pressed("N4"):
            gripper1.set_reverse(False)
            gripper1.on(100)
        else:
            gripper1.off()
        

# Instantiate DC motors
lift = encoder_motor_class("M4", "INDEX1") # using encoder for spacific position of lift functions
cooling = dc_motor("DC2")
gripper1 = dc_motor("DC1")
conveyer = dc_motor("DC5")
entrance_feed = dc_motor("DC6")
feeder = dc_motor("DC7")
bl_1 = brushless_motor("BL1")
bl_2 = brushless_motor("BL2")
shooter = smartservo_class("M1", "INDEX1") # only for angles
laser = dc_motor("DC8")
front_input = dc_motor("DC3")

while True:
    cooling.set_reverse(True)
    cooling.on(100)
    if power_manage_module.is_auto_mode():
        pass
    else:
        if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            runtime.change_mode()
        else:
            if runtime.CTRL_MODE == 0:
                shoot_mode.control_button()
                runtime.move_1()
            else:
                gripper_mode.control_button()
                runtime.move_2()