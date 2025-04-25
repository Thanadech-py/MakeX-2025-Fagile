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

left_forward_wheel = encoder_motor_class("M2", "INDEX1")
right_forward_wheel = encoder_motor_class("M3", "INDEX1")
left_back_wheel = encoder_motor_class("M5", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")

MAX_SPEED = 255
SPEED_MULTIPLIER = 2.1
PID_SPEED_MULTIPLIER = 2.1
BL_POWER = 80

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0
    
    def compute(self, measured_value, dt):
        # Calculate the error
        error = self.setpoint - measured_value
        
        # Integral term
        self.integral += error * dt
        
        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        # PID terms
        P = self.Kp * error
        I = self.Ki * self.integral
        D = self.Kd * derivative

        # Save the previous error for next iteration
        self.prev_error = error
        
        # Return the output (adjusted speed)
        return P + I + D

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0

class motors:
    
    def drive(lf: int, lb: int, rf: int, rb: int):
        # Setting motor speed based on calculated PID output
        left_back_wheel.set_speed(lb)        # LEFT back 
        right_back_wheel.set_speed(-rb)      # RIGHT BACK  
        right_forward_wheel.set_speed(-(rf)) # RIGHT FORWARD
        left_forward_wheel.set_speed(lf)     # LEFT BACK
    
    def stop():
        motors.drive(0, 0, 0, 0)

class util:
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)

class holonomic:    

    pids = {
        "lf": PID(Kp=1, Ki=0, Kd=0),
        "lb": PID(Kp=1, Ki=0, Kd=0),
        "rf": PID(Kp=0.9, Ki=0, Kd=0),
        "rb": PID(Kp=0.9, Ki=0, Kd=0),
    }
    
    def drive(vx, vy, wL, deadzone=10, pid=True):
        global SPEED_MULTIPLIER, PID_SPEED_MULTIPLIER
        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0

        # Ensure the correct speed multiplier
        multiplier = PID_SPEED_MULTIPLIER if pid else SPEED_MULTIPLIER
            
        # Calculation for the wheel speed (Desired speed for each wheel)
        vFL = (vx + (vy * 1.2) + wL) * multiplier
        vFR = (-(vx) + (vy * 1.2) - wL) * multiplier
        vBL = (-(vx) + (vy * 1.2) + wL) * multiplier
        vBR = (vx + (vy * 1.2) - wL) * multiplier
        
        if pid:            
            vFL = holonomic.pids["lf"].compute(-left_forward_wheel.get_value("speed"), dt)
            vBL = holonomic.pids["lb"].compute(-left_back_wheel.get_value("speed"), dt)
            vFR = holonomic.pids["rf"].compute(right_forward_wheel.get_value("speed"), dt)
            vBR = holonomic.pids["rb"].compute(right_back_wheel.get_value("speed"), dt)

        # Restrict values to max speed
        vFL = util.restrict(vFL, -MAX_SPEED, MAX_SPEED)
        vFR = util.restrict(vFR, -MAX_SPEED, MAX_SPEED)
        vBL = util.restrict(vBL, -MAX_SPEED, MAX_SPEED)
        vBR = util.restrict(vBR, -MAX_SPEED, MAX_SPEED)

        # Send the calculated speed values to the motors
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
        
class Auto():
    def right():
        pass
    def left():
        pass  
    
class Motors:
    # Default DC port
    DC_port = "DC1"
    BL_port = "BL1"
    
    # Initialize DC motor with a specific port
    def __init__(self, port: str) -> None:
        self.DC_port = port
        self.BL_port = port 
        
    # Method to turn on the DC Motor
    def onDC(self, Power: int) -> None:
        power = -Power if self.reverse else Power
        power_expand_board.set_power(self.dc_port, power)
    # Method to turn on Blushless Motor
    def onBL(self) -> None:
        power_expand_board.set_power(self.BL_port, BL_POWER)        
    # Method to turn off the DC Motor
    def offDC(self) -> None:
        power_expand_board.stop(self.DC_port)
    
    #Method to turn off Blushless Motor
    def offBL(self) -> None:
        power_expand_board.stop(self.BL_port)
        
class Mode:
    CTRL_MODE = 0
    
    ENABLE = True
    def move_1():
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            holonomic.drive(-gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), -gamepad.get_joystick("Rx"), pid=True)
        else:
            motors.drive(0,0,0,0)
            
    def move_2():
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            holonomic.drive(gamepad.get_joystick("Lx"), -gamepad.get_joystick("Ly"), -gamepad.get_joystick("Rx"), pid=True)
        else:
            motors.drive(0,0,0,0)
            
    def change_mode():
        if novapi.timer() > 0.9:
            if runtime.CTRL_MODE == 0:
                runtime.CTRL_MODE = 1
            else:
                runtime.CTRL_MODE = 0
            novapi.reset_timer()
        
    
while True:
    if power_manage_module.is_auto_mode():
        pass
    else:
        if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            Mode.change_mode()
        else:
            if Mode.CTRL_MODE == 0:
                Mode.move_1()
            else:
                Mode.move_2()