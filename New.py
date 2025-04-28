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

# Motor instances
left_forward_wheel = encoder_motor_class("M2", "INDEX1")
right_forward_wheel = encoder_motor_class("M3", "INDEX1")
left_back_wheel = encoder_motor_class("M5", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")

# Constants
MAX_SPEED = 255
SPEED_MULTIPLIER = 2.1
PID_SPEED_MULTIPLIER = 2.1
BL_POWER = 80

# Timer for dt calculation
_last_time = time.time()

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0

    def compute(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error

        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0

class Motors:
    @staticmethod
    def drive(lf: int, lb: int, rf: int, rb: int):
        left_back_wheel.set_speed(lb)
        right_back_wheel.set_speed(-rb)
        right_forward_wheel.set_speed(-rf)
        left_forward_wheel.set_speed(lf)

    @staticmethod
    def stop():
        Motors.drive(0, 0, 0, 0)

class Util:
    @staticmethod
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)

class Holonomic:
    pids = {
        "lf": PID(Kp=1, Ki=0, Kd=0),
        "lb": PID(Kp=1, Ki=0, Kd=0),
        "rf": PID(Kp=0.9, Ki=0, Kd=0),
        "rb": PID(Kp=0.9, Ki=0, Kd=0),
    }

    @staticmethod
    def drive(vx, vy, wL, deadzone=10, pid=True):
        global _last_time

        now = time.time()
        dt = now - _last_time
        _last_time = now

        if abs(vx) < deadzone:
            vx = 0
        if abs(vy) < deadzone:
            vy = 0
        if abs(wL) < deadzone:
            wL = 0

        multiplier = PID_SPEED_MULTIPLIER if pid else SPEED_MULTIPLIER

        vFL = (vx + (vy * 1.2) + wL) * multiplier
        vFR = (-(vx) + (vy * 1.2) - wL) * multiplier
        vBL = (-(vx) + (vy * 1.2) + wL) * multiplier
        vBR = (vx + (vy * 1.2) - wL) * multiplier

        if pid:
            vFL = Holonomic.pids["lf"].compute(-left_forward_wheel.get_value("speed"), dt)
            vBL = Holonomic.pids["lb"].compute(-left_back_wheel.get_value("speed"), dt)
            vFR = Holonomic.pids["rf"].compute(right_forward_wheel.get_value("speed"), dt)
            vBR = Holonomic.pids["rb"].compute(right_back_wheel.get_value("speed"), dt)

        vFL = Util.restrict(vFL, -MAX_SPEED, MAX_SPEED)
        vFR = Util.restrict(vFR, -MAX_SPEED, MAX_SPEED)
        vBL = Util.restrict(vBL, -MAX_SPEED, MAX_SPEED)
        vBR = Util.restrict(vBR, -MAX_SPEED, MAX_SPEED)

        Motors.drive(vFL, vBL, vFR, vBR)

    @staticmethod
    def move_forward(power):
        Holonomic.drive(0, power, 0)

    @staticmethod
    def move_backward(power):
        Holonomic.drive(0, -power, 0)

    @staticmethod
    def slide_right(power):
        Holonomic.drive(power, 0, 0)

    @staticmethod
    def slide_left(power):
        Holonomic.drive(-power, 0, 0)

    @staticmethod
    def turn_right(power):
        Holonomic.drive(0, 0, power)

    @staticmethod
    def turn_left(power):
        Holonomic.drive(0, 0, -power)

class Auto:
    @staticmethod
    def right():
        pass

    @staticmethod
    def left():
        pass

class PowerMotor:
    def __init__(self, port: str) -> None:
        self.DC_port = port
        self.BL_port = port
        self.reverse = False

    def onDC(self, Power: int) -> None:
        power = -Power if self.reverse else Power
        power_expand_board.set_power(self.DC_port, power)

    def onBL(self) -> None:
        power_expand_board.set_power(self.BL_port, BL_POWER)

    def offDC(self) -> None:
        power_expand_board.stop(self.DC_port)

    def offBL(self) -> None:
        power_expand_board.stop(self.BL_port)

class Mode:
    CTRL_MODE = 0
    ENABLE = True

    @staticmethod
    def move_1():
        if abs(gamepad.get_joystick("Lx")) > 20 or abs(gamepad.get_joystick("Ly")) > 20 or abs(gamepad.get_joystick("Rx")) > 20:
            Holonomic.drive(-gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), -gamepad.get_joystick("Rx"), pid=True)
        else:
            Motors.stop()

    @staticmethod
    def move_2():
        if abs(gamepad.get_joystick("Lx")) > 20 or abs(gamepad.get_joystick("Ly")) > 20 or abs(gamepad.get_joystick("Rx")) > 20:
            Holonomic.drive(gamepad.get_joystick("Lx"), -gamepad.get_joystick("Ly"), -gamepad.get_joystick("Rx"), pid=True)
        else:
            Motors.stop()

    @staticmethod
    def change_mode():
        if novapi.timer() > 0.9:
            if Mode.CTRL_MODE == 0:
                Mode.CTRL_MODE = 1
            else:
                Mode.CTRL_MODE = 0
            novapi.reset_timer()

# Main loop
while True:
    if power_manage_module.is_auto_mode():
        pass  # Auto mode logic can be added here
    else:
        if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            Mode.change_mode()
        else:
            if Mode.CTRL_MODE == 0:
                Mode.move_1()
            else:
                Mode.move_2()
