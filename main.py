"""Fragile Main Program
This is the main program of the Fragile Team From SuksanareeWittaya where we participate in MakeX Robotics Competition. 
The program includes holonomic drive control,
DC motor and brushless motor management, and runtime operations for shooting and gripper control.
"""

# Import necessary modules
import novapi
import math
from time import sleep
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module
from mbuild.led_matrix import led_matrix_class

status = led_matrix_class("PORT3", "INDEX1")
class PID:
    def __init__(self, Kp,  Ki, Kd, setpoint=0):
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


class util:
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)
        
class holonomic():    
    
    # Maximum speed for the motors
    MAX_SPEED = 500  

    left_front = encoder_motor_class("M3", "INDEX1")
    right_front = encoder_motor_class("M1", "INDEX1")
    left_back = encoder_motor_class("M4", "INDEX1")
    right_back = encoder_motor_class("M5", "INDEX1")
    
    #tuned PID values for each motor
    pids = {
        "lf": PID(Kp=0.9125, Ki=0, Kd=0.55),
        "lb": PID(Kp=0.65, Ki=0, Kd=0.0125),
        "rf": PID(Kp=1.0625, Ki=0, Kd=0),
        "rb": PID(Kp=0.6, Ki=0.0018, Kd=0.05),
    }


    def motordrive(lf: int, lb: int, rf: int, rb: int):
        holonomic.left_front.set_speed(lf)
        holonomic.left_back.set_speed(lb)
        holonomic.right_front.set_speed(-rf)
        holonomic.right_back.set_speed(-rb)  
        
    
    def motorstop():
        holonomic.motordrive(0, 0, 0, 0)
    
    def drive(vx, vy, wL, deadzone=5):
        if abs(vx) < deadzone: vx = 0
        if abs(vy) < deadzone: vy = 0
        if abs(wL) < deadzone: wL = 0

        multiplier = 3 #PID Speed Multiplier

        vFL = (vx + (vy * 2) + wL) * multiplier
        vFR = (-(vx) + (vy * 2) - wL) * multiplier
        vBL = (-(vx) + (vy * 2) + wL) * multiplier
        vBR = (vx + (vy * 2) - wL) * multiplier
        
        
        holonomic.pids["lf"].set_setpoint(vFL)
        holonomic.pids["lb"].set_setpoint(vBL)
        holonomic.pids["rf"].set_setpoint(vFR)
        holonomic.pids["rb"].set_setpoint(vBR)

        vFL_out = holonomic.pids["lf"].update(-holonomic.left_front.get_value("speed"))
        vBL_out = holonomic.pids["lb"].update(-holonomic.left_back.get_value("speed"))
        vFR_out = holonomic.pids["rf"].update(holonomic.right_front.get_value("speed"))
        vBR_out = holonomic.pids["rb"].update(holonomic.right_back.get_value("speed"))

    
        vFL_out = util.restrict(vFL_out, -holonomic.MAX_SPEED, holonomic.MAX_SPEED)
        vFR_out = util.restrict(vFR_out, -holonomic.MAX_SPEED, holonomic.MAX_SPEED)
        vBL_out = util.restrict(vBL_out, -holonomic.MAX_SPEED, holonomic.MAX_SPEED)
        vBR_out = util.restrict(vBR_out, -holonomic.MAX_SPEED, holonomic.MAX_SPEED)

    
        holonomic.motordrive(vFL_out, vBL_out, vFR_out, vBR_out)


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
    
    def stop():
        holonomic.drive(0, 0, 0)
        
        
class Auto_backend:
    def __init__(self, r=0.03, L=0.14, W=0.20): #Units in meters
        self.r = r
        self.L = L
        self.W = W

    def update(self):
        rpm_to_rad = 2 * math.pi / 60

        W1 = holonomic.left_front.get_value("speed") * rpm_to_rad
        W2 = holonomic.right_front.get_value("speed") * rpm_to_rad
        W3 = holonomic.left_back.get_value("speed") * rpm_to_rad
        W4 = holonomic.right_back.get_value("speed") * rpm_to_rad

        self.vx = (self.r / 4) * (W1 + W2 + W3 + W4)   # m/s
        self.vy = (self.r / 4) * (-W1 + W2 + W3 - W4)  # m/s
        self.wL = (self.r / (4 * (self.L + self.W))) * (-W1 + W2 - W3 + W4)  # rad/s

        return self.vx, self.vy, self.wL

    def move_forward(self, distance, power):
        holonomic.move_forward(power)
        target_x = self.vx * novapi.timer() + distance
        while abs(target_x - self.vx * novapi.timer()) > 0.01:
            novapi.delay(10)
        holonomic.stop()
        
   
class dc_motor:
    # Default DC port
    dc_port = "DC1"
    # Default direction (not reversed)
    reverse = False
    
    # Initialize DC motor with a specific port
    def __init__(self, port: str) -> None:
        self.dc_port = port
        
    # Method to control the DC motor
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

    BL_POWER_MAX = 110

    # Initialize brushless motor with a specific port
    def __init__(self, port: str) -> None:
        self.bl_port = port
        
    # Method to turn on the brushless motor
    def on(self) -> None:
        power_expand_board.set_power(self.bl_port, self.BL_POWER_MAX)
        
    # Method to turn off the brushless motor
    def off(self) -> None:
        power_expand_board.stop(self.bl_port)
      

class runtime:
    # Define control mode
    CTRL_MODE = 0

    def move():
        if gamepad.is_key_pressed("Up"):
            holonomic.move_forward(holonomic.MAX_SPEED)
        elif gamepad.is_key_pressed("Down"):
            holonomic.move_backward(holonomic.MAX_SPEED)
        elif gamepad.is_key_pressed("Left"):
            holonomic.turn_left(holonomic.MAX_SPEED)
        elif gamepad.is_key_pressed("Right"):
            holonomic.turn_right(holonomic.MAX_SPEED)
        elif abs(gamepad.get_joystick("Lx")) > 20:
            holonomic.drive(-gamepad.get_joystick("Lx"), 0, 0)
        else :
            holonomic.stop()

    def change_mode():
        if novapi.timer() > 0.9:
            if runtime.CTRL_MODE == 0:
                runtime.CTRL_MODE = 1
            else:
                runtime.CTRL_MODE = 0
            novapi.reset_timer()
    
    def shoot_peem():
        if gamepad.get_joystick("Ry") > 20:
            entrance_feed.on(70, True)
            feeder.on(70, True)
            front_input.on(100, True)
            disc_stock.on(100, False)
        elif gamepad.get_joystick("Ry") < -20:
            entrance_feed.on(70, False)
            feeder.on(70, False)
            front_input.on(100, False)
            disc_stock.on(100, True)
        else:
            entrance_feed.off()
            feeder.off()
            front_input.off()
            disc_stock.off()
        #Shooter control
        if gamepad.is_key_pressed("R1") or gamepad.is_key_pressed("L1"):
            bl_2.on()
        else:
            bl_2.off()
        
        angle = 0
        if gamepad.is_key_pressed("N2"):            
            angle = angle - 58
        elif gamepad.is_key_pressed("N3"):
            angle = angle + 28
        else:
            angle = angle_left.get_value("angle") and angle_right.get_value("angle")
            

        # Lock angle position by setting both servos to the current angle
        angle_left.move_to(angle, 10)
        angle_right.move_to(angle, 10)
    
    def gripper_peem():
        if gamepad.is_key_pressed("N2"):
            lift.set_power(-100)
        elif gamepad.is_key_pressed("N3"):
            lift.set_power(20)
        else:
            lift.set_speed(0)
        
        if gamepad.is_key_pressed("N1"):
            gripper.on(60, True)
        elif gamepad.is_key_pressed("N4"):
            gripper.on(60, False)
        else:
            gripper.off()
            
        if gamepad.is_key_pressed("R1"):
            left_block.on(100, True)
            right_block.on(100, False)
        elif gamepad.is_key_pressed("L1"):
            left_block.on(100, False)
            right_block.on(100, True)
        else:
            left_block.off()
            right_block.off()

class Auto:
    def run():
        Auto_backend.move_forward(0.5, 200)
        
        return holonomic.motorstop()


#Block and Cube Management System
entrance_feed = dc_motor("DC1")
feeder = dc_motor("DC2")
front_input = dc_motor("DC3")
disc_stock = dc_motor("DC6")
#lift and gripper
lift = encoder_motor_class("M6", "INDEX1")
gripper = dc_motor("DC7")
#shooting
bl_2 = brushless_motor("BL1")
#shooting angle
angle_right = smartservo_class("M1", "INDEX1")
angle_left = smartservo_class("M1", "INDEX2") # only for angles
#utility
left_block = dc_motor("DC4")
right_block = dc_motor("DC5")

while True:
    if power_manage_module.is_auto_mode():
        status.show("A", wait=False)
        Auto.run()
    else:
        runtime.move()
        if gamepad.is_key_pressed("R2"):
            runtime.change_mode()
        else:
            if runtime.CTRL_MODE == 0:
                status.show("S", wait=False)
                runtime.shoot_peem()
            else:
                status.show("G", wait=False)
                runtime.gripper_peem()
        