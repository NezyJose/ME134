from XRPLib.differential_drive import DifferentialDrive
from machine import Timer
from XRPLib.board import Board
import time, math, gc, os

board = Board.get_default_board()

drivetrain = DifferentialDrive.get_default_differential_drive()

gc.collect() # empty RAM

# Data collection
data = [] # temp. storage
data_interval = 20 #ms
filename = "data_nofilter1.csv"

if filename in os.listdir():
    os.remove(filename)
    print(f"{filename} deleted.")
else:
    print(f"{filename} does not exist.")

# Hardware timer for pose_udpate
hardware_timer_period = 0.1 #s

class PositionEstimation:
    def __init__(self):
        self.track_width = 15.5 #cm
        self.wheel_diam = 6 #cm
        self.RPMtoCMPS = (math.pi * self.wheel_diam) / 60     # Covert from RPM to cm/s
        self.CMPStoRPM = 60 / (math.pi * self.wheel_diam)     # Covert from cm/s to RPM
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_data_time = time.ticks_ms()
    def pose_update(self):
        right_motor_speed = drivetrain.right_motor.get_speed()*self.RPMtoCMPS
        left_motor_speed = drivetrain.left_motor.get_speed()*self.RPMtoCMPS
        
        # calculcate forward kinematics model and update x,y, and theta
        # ...
        if abs(right_motor_speed - left_motor_speed) < 1e-5: #straight
            # treat it as a straight line
            speed = (right_motor_speed + left_motor_speed) / 2
            self.x = self.x + speed * math.cos(self.theta) * hardware_timer_period
            self.y = self.y + speed * math.sin(self.theta) * hardware_timer_period
            self.theta_new = self.theta
        else: #turning
            radius = self.track_width / 2 * (right_motor_speed + left_motor_speed) / (right_motor_speed - left_motor_speed)
            ang_speed = (right_motor_speed - left_motor_speed) / self.track_width
            speed = ang_speed * radius
            self.x = self.x - radius * math.sin(self.theta) + radius * math.sin(self.theta + ang_speed * hardware_timer_period)
            self.y = self.y + radius * math.cos(self.theta) - radius * math.cos(self.theta + ang_speed * hardware_timer_period)
            self.theta = self.theta + ang_speed * hardware_timer_period


        # Store every data_interval, robot pose in data[]
        current_time = time.ticks_ms() 
        if time.ticks_diff(current_time, self.last_data_time) >= data_interval:
            print(current_time,self.x,self.y, self.theta*180/math.pi)
            data.append([
                self.x,
                self.y,
                self.theta
            ])
            self.last_data_time = current_time
    def set_motor_target_speeds(self, right_cmps,left_cmps):
        drivetrain.left_motor.set_speed(left_cmps)
        drivetrain.right_motor.set_speed(right_cmps)

kinematics = PositionEstimation()


def execute_traj():
    motion_sequence=[
        (0,0,2),(100,100,3),(-200,200,2),(150,150,1),(-150,150,1),(50,50,3),(-200,200,1),(100,100,2),(-150,150,3),(100,100,1),(-150,150,1),(50,50,1)
    ]
    for left,right,duration in motion_sequence:
        kinematics.set_motor_target_speeds(left,right)
        time.sleep(duration)
        
try:
    print("Waiting for start button...")
    board.wait_for_button()  # Wait for the button to start
    print("Started! Press the button again to stop.")

    timer=Timer()

    timer.init(
        period=100,
        mode=Timer.PERIODIC,
        callback=lambda t: kinematics.pose_update()
    )

    execute_traj()
    print("done")

    # After experiment, write into file
    with open(filename, "w") as f:
        f.write('x,y,theta\n')
        for row in data:
            f.write(f"{row[0]},{row[1]},{row[2]}\n")
    print("experiment completed")
    drivetrain.stop()

except KeyboardInterrupt:
    drivetrain.stop()
