from XRPLib.differential_drive import DifferentialDrive
from machine import Timer
from XRPLib.board import Board
from XRPLib.imu import IMU
import time, math, gc, os

board = Board.get_default_board()

drivetrain = DifferentialDrive.get_default_differential_drive()

imu=IMU.get_default_imu()

gc.collect() # empty RAM

# Data collection
data = [] # temp. storage
data_interval = 20 #ms
filename = "data_filter1.csv"
test=True

if filename in os.listdir():
    os.remove(filename)
    print(f"{filename} deleted.")
else:
    print(f"{filename} does not exist.")

def matrix_add(A,B):
    return [[A[i][j]+B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

def invert_diagonal_matrix(M):
    return [[1.0/M[i][j] if i==j and M[i][j] !=0 else 0.0 for j in range(len(M))] for i in range(len(M))]

def matrix_multiply(A,B):
    return [ 
        [sum(A[i][k]*B[k][j] for k in range(len(B))) for j in range(len(B[0]))]
        for i in range(len(A))
    ]
# Hardware timer for pose_udpate
hardware_timer_period = 0.1 #s

class PositionEstimation:
    def __init__(self):
        self.track_width = 15.5 #cm
        self.wheel_diam = 6 #cm
        self.RPMtoCMPS = (math.pi * self.wheel_diam) / 60     # Covert from RPM to cm/s
        self.CMPStoRPM = 60 / (math.pi * self.wheel_diam)     # Covert from cm/s to RPM
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_data_time = time.ticks_ms()

        self.x_x=self.x_y=self.x_theta=0.0
        self.z_x=self.z_y=self.z_theta=0.0
        self.old_theta=0.0
        self.gyro_theta=0.0

        self.P= [
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.0, 0.1]
        ]
        self.Q= [
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.0, 0.1]
        ]
        self.R= [
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.0, 0.1]
        ]
        
    def update_step(self):
        z_gyro=imu.get_gyro_z_rate()*(math.pi/180000)
        self.gyro_theta+=z_gyro*hardware_timer_period

        innovation= self.gyro_theta-self.x_theta
        P_theta=self.P[2][2]
        R_theta=self.R[2][2]
        K_theta=P_theta/(P_theta+R_theta)

        self.theta = self.x_theta + K_theta * innovation
        self.x_theta = self.theta
        self.P[2][2] = (1-K_theta)*self.P[2][2]

        current_time = time.ticks_ms() 
        print(current_time,self.x,self.y, self.old_theta*180/math.pi, self.gyro_theta*180/math.pi, self.x_theta*180/math.pi)
        if time.ticks_diff(current_time, self.last_data_time) >= data_interval:
            data.append([
                current_time,
                self.x,
                self.y,
                self.old_theta*180/math.pi,
                self.gyro_theta*180/math.pi,
                self.x_theta*180/math.pi
            ])
            self.last_data_time = current_time
        
    def prediction_step(self):
        right_motor_speed = drivetrain.right_motor.get_speed()*self.RPMtoCMPS
        left_motor_speed = drivetrain.left_motor.get_speed()*self.RPMtoCMPS
        
        # calculcate forward kinematics model and update x,y, and theta
        if abs(right_motor_speed - left_motor_speed) < 1e-5: #straight
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
        
        self.old_theta=self.theta
        self.P=matrix_add(self.P,self.Q)

        self.update_step()

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
        callback=lambda t: kinematics.prediction_step()
    )

    execute_traj()
    print("done")
    drivetrain.left_motor.set_speed(0)
    drivetrain.right_motor.set_speed(0)

    # After experiment, write into file
    with open(filename, "w") as f:
        f.write('current_time,x,y,old_theta,gyro_theta,x_theta\n')
        for row in data:
            f.write(f"{row[0]},{row[1]},{row[2]},{row[3]},{row[4]},{row[5]}\n")
    print("experiment completed")
    drivetrain.stop()
    

except KeyboardInterrupt:
    drivetrain.stop()
