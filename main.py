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

    MAX_SPEED = 255  # Maximum speed for the motors

    left_front = encoder_motor_class("M3", "INDEX1")
    right_front = encoder_motor_class("M1", "INDEX1")
    left_back = encoder_motor_class("M4", "INDEX1")
    right_back = encoder_motor_class("M5", "INDEX1")

    pids = {
        "lf": PID(Kp=1, Ki=0, Kd=0),
        "lb": PID(Kp=1, Ki=0, Kd=0),
        "rf": PID(Kp=0.9, Ki=0, Kd=0),
        "rb": PID(Kp=0.9, Ki=0, Kd=0),
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

        multiplier = 2.1 #PID Speed Multiplier

        vFL = (vx + (vy * 1.2) + wL) * multiplier
        vFR = (-(vx) + (vy * 1.2) - wL) * multiplier
        vBL = (-(vx) + (vy * 1.2) + wL) * multiplier
        vBR = (vx + (vy * 1.2) - wL) * multiplier
        
        
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
        

class auto_backend:
    # Robot physical constants
    radius = 0.03   # 30 mm wheel radius
    Length = 0.10   # Half of robot length (m)
    Width  = 0.08   # Half of robot width (m)

    @staticmethod
    def forward_kinematic():
        # Get wheel speeds (rad/s)
        w_fl = holonomic.left_front.get_value("speed")
        w_fr = holonomic.right_front.get_value("speed")
        w_rl = holonomic.left_back.get_value("speed")
        w_rr = holonomic.right_back.get_value("speed")

        # X-axis linear velocity (m/s)
        Vx = (auto_backend.radius / 4) * (w_fl + w_fr + w_rl + w_rr)

        # Y-axis linear velocity (m/s)
        Vy = (auto_backend.radius / 4) * (-w_fl + w_fr + w_rl - w_rr)

        # Angular velocity around Z-axis (rad/s)
        Wz = (auto_backend.radius / (4 * (auto_backend.Length + auto_backend.Width))) * (-w_fl + w_fr - w_rl + w_rr)

        return Vx, Vy, Wz
    
    def move_forward_distance(distance: float, power: int):
        target_distance = distance  # meters
        Kp = 50  # Proportional gain for distance control
        tolerance = 0.01  # Acceptable error (m)

        traveled_distance = 0.0
        prev_time = novapi.timer()

        while abs(traveled_distance - target_distance) > tolerance:
            current_time = novapi.timer()
            dt = current_time - prev_time
            prev_time = current_time

            # อ่านความเร็วจากล้อ
            Vx, Vy, Wz = auto_backend.forward_kinematic()

            # เดินหน้า ใช้แกน X (เพราะสูตรคุณกำหนด Vx คือหน้า/หลัง)
            traveled_distance += Vx * dt

            # คำนวณ error
            error = target_distance - traveled_distance
            control_signal = Kp * error

            # จำกัดกำลังไม่ให้เกิน max power
            control_signal = util.restrict(control_signal, -power, power)

            # ชะลอความเร็วใกล้ถึงเป้าหมาย
            if abs(error) < 0.05:
                control_signal *= 0.5

            # ส่งคำสั่งไปที่ระบบขับเคลื่อน
            holonomic.move_forward(control_signal)

        holonomic.stop()
        sleep(0.1)
        
    @staticmethod
    def slide_left_distance(distance: float, power: int):
        target_distance = distance
        Kp = 50
        tolerance = 0.01
        traveled_distance = 0.0
        prev_time = novapi.timer()

        while abs(traveled_distance - target_distance) > tolerance:
            current_time = novapi.timer()
            dt = current_time - prev_time
            prev_time = current_time

            Vx, Vy, Wz = auto_backend.forward_kinematic()
            traveled_distance += abs(Vy) * dt  # ใช้ Vy เพราะเลื่อนข้าง

            error = target_distance - traveled_distance
            control_signal = Kp * error
            control_signal = util.restrict(control_signal, -power, power)

            if abs(error) < 0.05:
                control_signal *= 0.5

            holonomic.slide_left(control_signal)
        holonomic.stop()
        sleep(0.1)
    @staticmethod
    def turn_left_angle(angle_deg: float, power: int):
        target_angle = math.radians(angle_deg)
        Kp = 100
        tolerance = math.radians(2)

        turned_angle = 0.0
        prev_time = novapi.timer()

        while abs(turned_angle - target_angle) > tolerance:
            current_time = novapi.timer()
            dt = current_time - prev_time
            prev_time = current_time

            Vx, Vy, Wz = auto_backend.forward_kinematic()
            turned_angle += Wz * dt

            error = target_angle - turned_angle
            control_signal = Kp * error
            control_signal = util.restrict(control_signal, -power, power)

            if abs(error) < math.radians(5):
                control_signal *= 0.5

            holonomic.turn_left(control_signal)
        holonomic.stop()
        sleep(0.1)



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
        # angle_left.move_to(-36, 20)
        # angle_right.move_to(-36, 20)
        # left_block.on(100, True)
        # right_block.on(100, False)
        # auto_backend.move_forward_distance(9, 100)
        # auto_backend.slide_left_distance(9, 100)
        # auto_backend.move_forward_distance(4, 100)
        # sleep(0.9)
        # auto_backend.move_backward_distance(4, 100)
        # auto_backend.turn_left_angle(10, 100)
        # left_block.on(100, False)
        # right_block.on(100, True)
        # sleep(1000000000)
        pass

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
        status.show("AUTO", wait=False)
        Auto.run()
    else:
        runtime.move()
        if gamepad.is_key_pressed("R2"):
            runtime.change_mode()
        else:
            if runtime.CTRL_MODE == 0:
                status.show("SHOOT", wait=False)
                runtime.shoot_peem()
            else:
                status.show("GRIPPER", wait=False)
                runtime.gripper_peem()
        