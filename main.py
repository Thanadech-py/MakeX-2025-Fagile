# Import necessary modules
import novapi
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module

left_forward_wheel = encoder_motor_class("M2", "INDEX1")
right_forward_wheel = encoder_motor_class("M6", "INDEX1")
left_back_wheel = encoder_motor_class("M3", "INDEX1")
right_back_wheel = encoder_motor_class("M4", "INDEX1")

MAX_SPEED = 255
BL_POWER = 70

MAX_ANGLE = -62
MIN_ANGLE = 24

ANGLE = 0


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
    @staticmethod
    def drive(lf: int, lb: int, rf: int, rb: int):
        left_forward_wheel.set_speed(lf)
        left_back_wheel.set_speed(lb)
        right_forward_wheel.set_speed(-rf)
        right_back_wheel.set_speed(-rb)

    @staticmethod
    def stop():
        motors.drive(0, 0, 0, 0)

class util:
    @staticmethod
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)
        
class sensor:
    acc_x_vals = []
    acc_z_vals = []

    @staticmethod
    def filtered_acc(axis: str, size: int = 5):
        if axis == "X":
            sensor.acc_x_vals.append(novapi.get_acceleration("X"))
            if len(sensor.acc_x_vals) > size:
                sensor.acc_x_vals.pop(0)
            return sum(sensor.acc_x_vals) / len(sensor.acc_x_vals)
        elif axis == "Z":
            sensor.acc_z_vals.append(novapi.get_acceleration("Z"))
            if len(sensor.acc_z_vals) > size:
                sensor.acc_z_vals.pop(0)
            return sum(sensor.acc_z_vals) / len(sensor.acc_z_vals)
    
class holonomic:    
    pid = {
        "vx": PID(0.1021, 0.07, 0.03211),
        "vy": PID(0.15, 0.05, 0.02)
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
        vx = 5 * holonomic.pid["vx"].update(sensor.filtered_acc("Z"))
        holonomic.pid["vy"].set_setpoint(vy)
        vy = 5 * holonomic.pid["vy"].update(sensor.filtered_acc("X"))

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

class localization:
    # Robot state
    x = 0.0
    y = 0.0
    theta = 0.0

    # Timing
    last_time = novapi.timer()

    # Robot physical dimensions (adjust to your robot)
    wheel_radius = 0.04  # meters
    wheel_base = 0.18    # front-back wheel distance
    track_width = 0.15   # left-right wheel distance

    @staticmethod
    def clamp(val, min_val, max_val):
        return max(min(val, max_val), min_val)

    @staticmethod
    def get_linear_speed(motor):
        try:
            rpm = motor.get_speed()
            rpm = localization.clamp(rpm, -500, 500)  # reasonable range
            return (2 * math.pi * localization.wheel_radius * rpm) / 60
        except Exception as e:
            print("Speed read error:", e)
            return 0.0

    @staticmethod
    def update():
        current_time = novapi.timer()
        dt = current_time - localization.last_time
        if dt <= 0 or dt > 1:  # Skip if timer is invalid or large delay
            localization.last_time = current_time
            return

        localization.last_time = current_time

        try:
            imu_z = novapi.get_gyroscope("Z")
            imu_z = localization.clamp(imu_z, -500, 500)  # deg/sec clamp
        except Exception as e:
            print("Gyro read error:", e)
            imu_z = 0

        dtheta = math.radians(imu_z) * dt
        localization.theta += dtheta

        # Get speeds
        vFL = localization.get_linear_speed(left_forward_wheel)
        vFR = localization.get_linear_speed(right_forward_wheel)
        vBL = localization.get_linear_speed(left_back_wheel)
        vBR = localization.get_linear_speed(right_back_wheel)

        # Inverse kinematics for robot-frame velocities
        vx = (vFL + vFR + vBL + vBR) / 4
        vy = (-vFL + vFR + vBL - vBR) / 4
        omega = (-vFL + vFR - vBL + vBR) / (4 * (localization.wheel_base + localization.track_width))

        # Global frame transformation
        cos_t = math.cos(localization.theta)
        sin_t = math.sin(localization.theta)

        global_vx = vx * cos_t - vy * sin_t
        global_vy = vx * sin_t + vy * cos_t

        localization.x += global_vx * dt
        localization.y += global_vy * dt

    @staticmethod
    def get_position():
        return localization.x, localization.y, math.degrees(localization.theta)



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
    def move_1():
        if gamepad.is_key_pressed("Up"):
            holonomic.move_forward(MAX_SPEED)
        elif gamepad.is_key_pressed("Down"):
            holonomic.move_backward(MAX_SPEED)
        elif gamepad.is_key_pressed("Left"):
            holonomic.turn_left(MAX_SPEED)
        elif gamepad.is_key_pressed("Right"):
            holonomic.turn_right(MAX_SPEED)
        elif abs(gamepad.get_joystick("Lx")) > 20:
            holonomic.drive(-gamepad.get_joystick("Lx"), 0, 0)
        else:
            motors.drive(0,0,0,0)
    def move_2():
        if gamepad.is_key_pressed("Up"):
            holonomic.slide_right(MAX_SPEED)
        elif gamepad.is_key_pressed("Down"):
            holonomic.slide_left(MAX_SPEED)
        elif gamepad.is_key_pressed("Left"):
            holonomic.turn_left(MAX_SPEED)
        elif gamepad.is_key_pressed("Right"):
            holonomic.turn_right(MAX_SPEED)
        elif abs(gamepad.get_joystick("Lx")) > 20:
            holonomic.drive(0, gamepad.get_joystick("Lx"), 0)
        else :
            holonomic.drive(0,0,0,0)

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
        if gamepad.get_joystick("Ry") > 40:
            entrance_feed.set_reverse(False)
            entrance_feed.on(100)
            feeder.set_reverse(True)
            feeder.on(100)
        elif gamepad.get_joystick("Ry") < -40:
            entrance_feed.set_reverse(True)
            entrance_feed.on(100)
            feeder.set_reverse(False)
            feeder.on(100)
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
        shooter.move_to(-gamepad.get_joystick("Ry") * 0.5, 10)
            

    
 
class gripper_mode:
    # Method to control various robot functions based on button inputs
    def control_button():
        if gamepad.is_key_pressed("N2"):
            lift.set_reverse(False)
            lift.on(100)
        elif gamepad.is_key_pressed("N3"):
            lift.set_reverse(True)
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
shooter = smartservo_class("M5", "INDEX1") # only for angles

while True:
    angle = shooter.get_value("angle")
    print("position: ", angle)

    localization.update()
    x, y, theta = localization.get_position()
    print("Position: X = {:.2f} m, Y = {:.2f} m, θ = {:.1f}°".format(x, y, theta))


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