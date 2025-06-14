import novapi
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module
import numpy as np

left_forward_wheel = encoder_motor_class("M2", "INDEX1")
right_forward_wheel = encoder_motor_class("M6", "INDEX1")
left_back_wheel = encoder_motor_class("M3", "INDEX1")
right_back_wheel = encoder_motor_class("M4", "INDEX1")

MAX_SPEED = 255

class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.Kp = kp  # Proportional gain
        self.Ki = ki  # Integral gain
        self.Kd = kd  # Derivative gain
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

class odometry:
    def __init__(self, left_front_encoder: encoder_motor_class, left_rear_encoder: encoder_motor_class, right_front_encoder: encoder_motor_class, right_rear_encoder: encoder_motor_class):
        self.left_front_encoder = left_front_encoder
        self.left_rear_encoder = left_rear_encoder
        self.right_front_encoder = right_front_encoder
        self.right_rear_encoder = right_rear_encoder
        self.wheel_radius = 0.04 # wheel radius (meter)
        self.wheel_distance = 0.35 # wheel distance from center of robot (meter)
        self.dt = 0.1 # time interval (second)
        self.x = 0.0 # x position (meter)
        self.y = 0.0 # y position (meter)
        self.theta = 0.0 # heading angle (radian)

    def get_gyro_angle(self):
        return novapi.get_yaw()
    
    def update_position(self):
        # get the rpm of each wheel
        left_front_rpm = self.left_front_encoder.get_value("speed")
        left_rear_rpm = self.left_rear_encoder.get_value("speed")
        right_front_rpm = self.right_front_encoder.get_value("speed")
        right_rear_rpm = self.right_rear_encoder.get_value("speed")
        
        # get the gyro angle
        theta = self.get_gyro_angle()

        # calculate the wheel speeds using the circumference of the wheel
        left_front_speed = (left_front_rpm * 2 * math.pi * self.wheel_radius) / 60
        left_rear_speed = (left_rear_rpm * 2 * math.pi * self.wheel_radius) / 60
        right_front_speed = (right_front_rpm * 2 * math.pi * self.wheel_radius) / 60
        right_rear_speed = (right_rear_rpm * 2 * math.pi * self.wheel_radius) / 60

        velocity_x = (left_front_speed + left_rear_speed + right_front_speed + right_rear_speed) / 4
        velocity_y = (left_front_speed + left_rear_speed + right_front_speed + right_rear_speed) / 4

        # update the position
        self.x += velocity_x * math.cos(self.theta) * self.dt
        self.y += velocity_y * math.sin(self.theta) * self.dt
        self.theta = theta
        
        return self.x, self.y, self.theta

    def get_position(self):
        return self.x, self.y, self.theta

class Motors:
    def drive(lf_speed, rf_speed, lb_speed, rb_speed):
        left_forward_wheel.set_speed(lf_speed)
        right_forward_wheel.set_speed(rf_speed)
        left_back_wheel.set_speed(-lb_speed)
        right_back_wheel.set_speed(-rb_speed)
    
    def stop():
        left_forward_wheel.set_speed(0)
        right_forward_wheel.set_speed(0)
        left_back_wheel.set_speed(0)
        right_back_wheel.set_speed(0)

class Utilities:
    def restrict(value, Min, Max):
        return max(min(value, Max), Min)
    
class HolonomicDrive:
    pid = {
        "Velocity_x": PID(0.5, 0.1, 0.05),
        "Velocity_y": PID(0.5, 0.1, 0.05),
        "Velocity_w": PID(0.5, 0.1, 0.05)
    }

    def drive(Velocity_x, Velocity_y, Velocity_w, deadzone=5):
        if math.fabs(Velocity_x) < math.fabs(deadzone):
            Velocity_x = 0
        if math.fabs(Velocity_y) < math.fabs(deadzone):
            Velocity_y = 0
        if math.fabs(Velocity_w) < math.fabs(deadzone):
            Velocity_w = 0

        lf_speed = Velocity_x + Velocity_y + Velocity_w
        rf_speed = -Velocity_x + Velocity_y - Velocity_w
        lb_speed = -Velocity_x + Velocity_y + Velocity_w
        rb_speed = Velocity_x + Velocity_y - Velocity_w

        lf_speed = Utilities.restrict(lf_speed, -MAX_SPEED, MAX_SPEED)
        rf_speed = Utilities.restrict(rf_speed, -MAX_SPEED, MAX_SPEED)
        rb_speed = Utilities.restrict(rb_speed, -MAX_SPEED, MAX_SPEED)
        lb_speed = Utilities.restrict(lb_speed, -MAX_SPEED, MAX_SPEED)  

        Motors.drive(lf_speed, rf_speed, lb_speed, rb_speed)
    
    #for automatic control (Old version)
    def forward(speed):
        HolonomicDrive.drive(0, speed, 0)
    def backward(speed):    
        HolonomicDrive.drive(0, -speed, 0)
    def left(speed):
        HolonomicDrive.drive(-speed, 0, 0)
    def right(speed):
        HolonomicDrive.drive(speed, 0, 0)
    def rotate_left(speed):
        HolonomicDrive.drive(0, 0, -speed)
    def rotate_right(speed):    
        HolonomicDrive.drive(0, 0, speed)

class Control:
    # Control modes
    CTRL_MODE = 0

    #Robot state
    ENABLED = True
    def ManualControl():
        if gamepad.is_key_pressed("Up"):
            HolonomicDrive.forward(MAX_SPEED)
        elif gamepad.is_key_pressed("Down"):
            HolonomicDrive.backward(MAX_SPEED)
        elif gamepad.is_key_pressed("Left"):
            HolonomicDrive.rotate_left(MAX_SPEED)
        elif gamepad.is_key_pressed("Right"):
            HolonomicDrive.rotate_right(MAX_SPEED)
        elif abs(gamepad.get_joystick("Lx")) > 20 or abs(gamepad.get_joystick("Ly")) > 20:
            HolonomicDrive.drive(-gamepad.get_joystick("Lx"), 0, 0)
        else:
            Motors.drive(0, 0, 0, 0)
    def DistanceControl():
        if gamepad.is_key_pressed("N1"):
            pass
        else:
            HolonomicDrive.drive(0, 0, 0)
    def change_mode():
        if novapi.timer() > 0.9:
            if Control.CTRL_MODE == 0:
                Control.CTRL_MODE = 1
            else:
                Control.CTRL_MODE = 0
            novapi.reset_timer()

while True:
    if power_manage_module.is_auto_mode():
        pass
    else:
        if gamepad.is_key_pressed("R2") and gamepad.is_key_pressed("L2"):
            Control.change_mode()
        else:
            if Control.CTRL_MODE == 0:
                Control.ManualControl()
            elif Control.CTRL_MODE == 1:
                Control.DistanceControl()
