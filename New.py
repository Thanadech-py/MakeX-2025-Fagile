# Import necessary modules
import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module
import time

left_forward_wheel = encoder_motor_class("M5", "INDEX1")
right_forward_wheel = encoder_motor_class("M2", "INDEX1")
left_back_wheel = encoder_motor_class("M4", "INDEX1")
right_back_wheel = encoder_motor_class("M3", "INDEX1")

MAX_SPEED = 255
BL_POWER = 70

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value (target)
        self.integral = 0  # Sum of errors over time
        self.previous_error = 0  # Previous error (used for derivative)

    def update(self, current_value):
        # Calculate the error (setpoint - current value)
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.previous_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0  # Reset the integral to avoid wind-up
        self.previous_error = 0  # Reset previous error to avoid a large derivative spike
        
class motors:
    
    def drive(lf: int, lb: int, rf: int, rb: int):
        left_forward_wheel.set_speed(lf)             # LEFT FORWARD
        left_back_wheel.set_speed(lb) # left back :DDDDD
        right_forward_wheel.set_speed(-rf)      # RIGHT FORWARD
        right_back_wheel.set_speed(-rb)  # RIGHT BACK  
   
    def stop():
        motors.drive(0, 0, 0, 0)
        
class util:
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)
    
class holonomic:    
    
    pid = {
        "vx": PID(0.5, 0.1, 0.1),
        "vy": PID(0.5, 0.1, 0.1)
    }

    def drive(self, vx, vy, wL, deadzone=5):
        # Create a deadzone so that if the joystick isn't moved perfectly,
        # the controller can still make the robot move perfectly.
        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0
            
        multiplier = 2 # SPEED MULTIPLIER
        
        self.pid["vx"].set_setpoint(vx)
        vx = 5 * self.pid["vx"].update(novapi.get_acceleration("X"))
        self.pid["vy"].set_setpoint(vy)
        vy = 5 * self.pid["vy"].update(novapi.get_acceleration("Z"))

        # Calculation for the wheel speed
        # Visit https://github.com/neumann-lab/holonomic-mecanum/blob/main/th.md for the formula
        vFL = (vx + vy + wL) * multiplier
        vFR = (-vx + vy - wL) * multiplier
        vBL = (-vx + vy + wL) * multiplier
        vBR = (vx + vy - wL) * multiplier

        # Velocity
        vFL = util.restrict(vFL, -MAX_SPEED, MAX_SPEED)
        vFR = util.restrict(vFR, -MAX_SPEED, MAX_SPEED)
        vBL = util.restrict(vBL, -MAX_SPEED, MAX_SPEED)
        vBR = util.restrict(vBR, -MAX_SPEED, MAX_SPEED)
        # Drive motor
        motors.drive(vFL, vBL, vFR, vBR)

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

robot_holonomic = holonomic()
      

class runtime:
    # Define control mode
    CTRL_MODE = 0
    
    # Robot state
    ENABLED = True
    def move_1():
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            robot_holonomic.drive((-gamepad.get_joystick("Lx")), (gamepad.get_joystick("Ly")), -(1.2 * gamepad.get_joystick("Rx")))
        else:
            motors.drive(0,0,0,0)

    def move_2():
        if math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ry")) > 20:
            robot_holonomic.drive((-gamepad.get_joystick("Ly")), (gamepad.get_joystick("Lx")), -(1.2 * gamepad.get_joystick("Ry")))
        else:
            motors.drive(0,0,0,0)
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
            entrance_feed.set_reverse(True)
            entrance_feed.on(60)
            feeder.set_reverse(False)
            feeder.on(60)
            front_input.set_reverse(True)
            front_input.on(60)
        elif gamepad.is_key_pressed("N3"):
            entrance_feed.set_reverse(False)
            entrance_feed.on(60)
            feeder.set_reverse(True)
            feeder.on(60)
            front_input.set_reverse(False)
            front_input.on(60)
        else:
            entrance_feed.off()
            feeder.off()
            front_input.off()
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
            lift.set_reverse(True)
            lift.on(100)
        elif gamepad.is_key_pressed("Down"):
            lift.set_reverse(False)
            lift.on(100)
        else:
            lift.off()
        if gamepad.is_key_pressed("N1"):
            gripper1.set_reverse(True)
            gripper1.on(100)

        elif gamepad.is_key_pressed("N4"):
            gripper1.set_reverse(False)
            gripper1.on(100)
        else:
            gripper1.off()
        

#Block and Cube Management System
entrance_feed = dc_motor("DC1")
feeder = dc_motor("DC2")
front_input = dc_motor("DC3")
#lift and gripper
lift = dc_motor("DC7")
gripper1 = dc_motor("DC8")
#shooting
bl_1 = brushless_motor("BL1")
bl_2 = brushless_motor("BL2")
#shooting angle
shooter = smartservo_class("M1", "INDEX1") # only for angles

while True:
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