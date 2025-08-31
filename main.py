# Import necessary modules
import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import gamepad
from mbuild.smartservo import smartservo_class

# Motor setup for 3 omni wheels
wheel1 = encoder_motor_class("M2", "INDEX1")  # Front
wheel2 = encoder_motor_class("M3", "INDEX1")  # Rear-left
wheel3 = encoder_motor_class("M4", "INDEX1")  # Rear-right

arm_enable = True


# Arm setup
arm = smartservo_class("M1", "INDEX1")  # Arm servo

MAX_SPEED = 35

# PID class (unchanged)
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value):
        error = self.setpoint - current_value
        P = self.Kp * error
        self.integral += error
        I = self.Ki * self.integral
        derivative = error - self.previous_error
        D = self.Kd * derivative
        self.previous_error = error
        return P + I + D

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

# Motor control
class motors:
    @staticmethod
    def drive(w1: int, w2: int, w3: int):
        wheel1.set_speed(w1)
        wheel2.set_speed(w2)
        wheel3.set_speed(w3)

    @staticmethod
    def stop():
        motors.drive(0, 0, 0)

class util:
    @staticmethod
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)


# Holonomic drive using 3 omni wheels
class holonomic:
    pid = {
        "vx": PID(0.1021, 0.07, 0.03211),
        "vy": PID(0.15, 0.05, 0.02)
    }

    @staticmethod
    def drive(vx, vy, wL, deadzone=5):
        if abs(vx) < deadzone:
            vx = 0
        if abs(vy) < deadzone:
            vy = 0
        if abs(wL) < deadzone:
            wL = 0

        R = 1  # rotational gain
        # Proper 3-omni kinematics
        w1 = vy + R * wL
        w2 = (-0.866 * vx - 0.5 * vy) + R * wL
        w3 = ( 0.866 * vx - 0.5 * vy) + R * wL

        # Scale to motor power
        w1 = util.restrict(w1 * MAX_SPEED, -MAX_SPEED, MAX_SPEED)
        w2 = util.restrict(w2 * MAX_SPEED, -MAX_SPEED, MAX_SPEED)
        w3 = util.restrict(w3 * MAX_SPEED, -MAX_SPEED, MAX_SPEED)

        motors.drive(int(w1), int(w2), int(w3))


# Gamepad movement
class runtime:

     # Define control mode
    CTRL_MODE = 0

    @staticmethod
    def move():

        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            holonomic.drive(gamepad.get_joystick("Lx"), -gamepad.get_joystick("Ly"), -gamepad.get_joystick("Rx"))
        else:
            motors.stop()

    def move_servo():
        if arm_enable == True:
                arm.move(25, 5)
                time.sleep(0.9)
                arm.move(-25, 5)
                time.sleep(0.9)  # Open position

    def change_mode():
        if novapi.timer() > 0.9:
            if runtime.CTRL_MODE == 0:
                runtime.CTRL_MODE = 1
            else:
                runtime.CTRL_MODE = 0
            novapi.reset_timer()


# Main loop
while True:
    if gamepad.is_key_pressed("N1"):
            runtime.change_mode()
    else:
        if runtime.CTRL_MODE == 0:
            runtime.move()
        else:
            runtime.move_servo()