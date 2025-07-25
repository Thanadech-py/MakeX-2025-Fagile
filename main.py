# Import necessary modules
import novapi
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module
#import time #for auto stages

#Define encoder motors for wheels
left_forward_wheel = encoder_motor_class("M2", "INDEX1")
right_forward_wheel = encoder_motor_class("M3", "INDEX1")
left_back_wheel = encoder_motor_class("M4", "INDEX1")
right_back_wheel = encoder_motor_class("M5", "INDEX1")

# Define constants
# Maximum speed for the motors
MAX_SPEED = 255
BL_POWER_MAX = 100
BL_POWER_MIN = 70

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
        output = max(min(P + I + D, MAX_SPEED), -MAX_SPEED)

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0  # Reset the integral to avoid wind-up
        self.previous_error = 0  # Reset previous error to avoid a large derivative spike
        
class motors:
    @staticmethod
    def drive(lf: int, lb: int, rf: int, rb: int):
        left_back_wheel.set_speed(lb) # left back :DDDDD
        right_back_wheel.set_speed(-rb)  # RIGHT BACK  
        right_forward_wheel.set_speed(-(rf))      # RIGHT FORWARD
        left_forward_wheel.set_speed(lf)             # LEFT BACK


    @staticmethod
    def stop():
        motors.drive(0, 0, 0, 0)

class util:
    @staticmethod
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)
        
class sensor:
    acc_x_vals = []
    acc_y_vals = []
    @staticmethod
    def filtered_acc(axis: str, size: int = 5):
        if axis == "X": # For moving left/right
            sensor.acc_x_vals.append(novapi.get_acceleration("X"))
            if len(sensor.acc_x_vals) > size:
                sensor.acc_x_vals.pop(0)
            return sum(sensor.acc_x_vals) / len(sensor.acc_x_vals)
        elif axis == "Y": #For moving forward/backward
            sensor.acc_y_vals.append(novapi.get_acceleration("Y"))
            if len(sensor.acc_y_vals) > size:
                sensor.acc_y_vals.pop(0)
            return sum(sensor.acc_y_vals) / len(sensor.acc_y_vals)
    
class holonomic:    
    pid = {
        "vx": PID(1, 0.06, 0.035),
        "vy": PID(1, 0.05, 0.9)

    }

    @staticmethod
    def drive(vx, vy, wL, deadzone=5):
        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0
            
        multiplier = 2
        
        holonomic.pid["vx"].set_setpoint(vx)
        vx = holonomic.pid["vx"].update(sensor.filtered_acc("Y"))
        holonomic.pid["vy"].set_setpoint(vy)
        vy = holonomic.pid["vy"].update(sensor.filtered_acc("X"))


        vFL = (vx + vy + wL) * multiplier
        vFR = (-vx + vy - wL) * multiplier
        vBL = (-vx + vy + wL) * multiplier
        vBR = (vx + vy - wL) * multiplier

        vFL = util.restrict(vFL, -MAX_SPEED, MAX_SPEED)
        vFR = util.restrict(vFR, -MAX_SPEED, MAX_SPEED)
        vBL = util.restrict(vBL, -MAX_SPEED, MAX_SPEED)
        vBR = util.restrict(vBR, -MAX_SPEED, MAX_SPEED)

        motors.drive(vFL, vBL, vFR, vBR)

    @staticmethod
    def move_forward(power):
        holonomic.drive(0, power, 0)

    @staticmethod
    def move_backward(power):
        holonomic.drive(0, -power, 0)

    @staticmethod
    def slide_right(power):
        holonomic.drive(power, 0, 0)

    @staticmethod
    def slide_left(power):
        holonomic.drive(-power, 0, 0)

    @staticmethod
    def turn_right(power):
        holonomic.drive(0, 0, power)

    @staticmethod
    def turn_left(power):
        holonomic.drive(0, 0, -power)    

class dc_motor:
    # Default DC port
    dc_port = "DC1"
    # Default direction (not reversed)
    reverse = False
    
    # Initialize DC motor with a specific port
    def __init__(self, port: str) -> None:
        self.dc_port = port
         
    # Method to turn control the DC motor
    def on(self, Power: int, rev: bool) -> None:
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
    def on_max(self) -> None:
        power_expand_board.set_power(self.bl_port, BL_POWER_MAX)
    
    def on_half(self) -> None:
        power_expand_board.set_power(self.bl_port, BL_POWER_MIN)
        
    # Method to turn off the brushless motor
    def off(self) -> None:
        power_expand_board.stop(self.bl_port)
      

class runtime:
    # Define control mode
    CTRL_MODE = 0
    
    # Robot state
    ENABLED = True
    def move():
        #control holonomic movement via gamepad arrow keys
        # if gamepad.is_key_pressed("Up"):
        #     holonomic.move_forward(MAX_SPEED)
        # elif gamepad.is_key_pressed("Down"):
        #     holonomic.move_backward(MAX_SPEED)
        # elif gamepad.is_key_pressed("Left"):
        #     holonomic.turn_left(MAX_SPEED)
        # elif gamepad.is_key_pressed("Right"):
        #     holonomic.turn_right(MAX_SPEED)
        # elif abs(gamepad.get_joystick("Lx")) > 20:
        #     holonomic.drive(-gamepad.get_joystick("Lx"), 0, 0)
        # else :
        #     holonomic.drive(0,0,0,0)
        
        #control holonomic movement via gamepad joysticks
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            holonomic.drive(-gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), -gamepad.get_joystick("Rx"))
        else:
            motors.drive(0,0,0,0)
                   
    def change_mode():
        if novapi.timer() > 0.9:
            if runtime.CTRL_MODE == 0:
                runtime.CTRL_MODE = 1
            else:
                runtime.CTRL_MODE = 0
            novapi.reset_timer()

#shooting mode
class shoot_mode:
    def control_button():
        if gamepad.is_key_pressed("N2"):
            entrance_feed.on(70, False)
            feeder.on(70, True)
            front_input.on(100, True)
        elif gamepad.is_key_pressed("N3"):
            entrance_feed.on(70, True)
            feeder.on(70, False)
            front_input.on(100, False)
        else:
            entrance_feed.off()
            feeder.off()
            front_input.off()
        #Shooter control
        if gamepad.is_key_pressed("R1"):
            bl_2.on_max()
        elif gamepad.is_key_pressed("L1"):
            bl_2.on_half()
        else:
            bl_2.off()

        #shooter_angle control
        if gamepad.is_key_pressed("Down"):
            angle_left.move(6, 10)
            angle_right.move(6, 10)
        elif gamepad.is_key_pressed("Up"):
            angle_left.move(-6,10)
            angle_right.move(-6, 10)
        else:
            pass
 
# Gripper control mode
class gripper_mode:
    def control_button():
        if gamepad.is_key_pressed("N2"):
            lift.set_power(-90)
        elif gamepad.is_key_pressed("N3"):
            lift.set_power(20)
        else:
            lift.set_speed(0)
        
        if gamepad.is_key_pressed("N1"):
            gripper.on(100, True)
        elif gamepad.is_key_pressed("N4"):
            gripper.on(100, False)
        else:
            gripper.off()

        if gamepad.is_key_pressed("R1"):
            left_motor_keeper.on(100, True)
            right_motor_keeper.on(100, False)
        elif gamepad.is_key_pressed("L1"):
            left_motor_keeper.on(100, False)
            right_motor_keeper.on(100, True)
        else:
            left_motor_keeper.off()
            right_motor_keeper.off()

class Auto:
    def run():
        pass
        

#Block and Cube Management System
entrance_feed = dc_motor("DC1")
feeder = dc_motor("DC2")
front_input = dc_motor("DC3")

#Auto keepper
left_motor_keeper = dc_motor("DC4")
right_motor_keeper = dc_motor("DC5")

#lift and gripper
lift = encoder_motor_class("M6", "INDEX1")
gripper = dc_motor("DC7")

#shooting
bl_2 = brushless_motor("BL1")

#shooting angle
angle_right = smartservo_class("M1", "INDEX2")
angle_left = smartservo_class("M1", "INDEX1")

#utility
Laser = dc_motor("DC8")


while True:
    if power_manage_module.is_auto_mode():
        Auto.run()
    else:
        runtime.move()
        if gamepad.is_key_pressed("R2"):
            runtime.change_mode()
        else:
            if runtime.CTRL_MODE == 0:
                shoot_mode.control_button()
                #Turn Laser on for shooting mode
                Laser.on(10, True)
            else:
                gripper_mode.control_button()
                #Turn Laser off for gripper mode
                Laser.off()