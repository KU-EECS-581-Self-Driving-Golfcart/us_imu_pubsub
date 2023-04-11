import datetime

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import Executor
from cadd_e_interface.msg import Ultrasonic, IMU

# Sensor Imports
import board
import busio
import adafruit_hcsr04
import adafruit_bno055

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(IMU, 'telemetry', 10)
        timer_period = 0.01 # seconds # TODO: Update
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=1600)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.heading = 0.0
        self.backup_heading = 0.0 # Last known heading before resetting sensor
        self.heading_offset = 0.0 # Last known heading before resetting sensor
        self.lin_acc = [0.0, 0.0, 0.0]
        self.ang_vel = [0.0, 0.0, 0.0]

    def timer_callback(self):
        reset = [False, False, False, False, False, False, False]
        msg = IMU()
        # Read sensor values
        lin_acc_read = self.sensor.linear_acceleration
        if lin_acc_read != None:
            #self.lin_acc = lin_acc_read
            if lin_acc_read[0] != None:
                self.lin_acc[0] = lin_acc_read[0]
            else:
                reset[0] = True
                print("NO LIN ACC DATA 1")

            if lin_acc_read[1] != None:
                self.lin_acc[1] = lin_acc_read[1]
            else:
                reset[1] = True
                print("NO LIN ACC DATA 2")

            if lin_acc_read[2] != None:
                self.lin_acc[2] = lin_acc_read[2]
            else:
                reset[2] = True
                print("NO LIN ACC DATA 3")
        else:
            print("NO LIN ACC DATA")
        

        euler_read = self.sensor.euler
        if euler_read != None and euler_read[0] != None:
            self.backup_heading = self.heading
            self.heading = (euler_read[0] + self.heading_offset) % 360.0
        else:
            reset[3] = True
            print("NO HEADING DATA")
        #if euler_read[0]:
        #    self.euler = euler_read

        ang_vel_read = self.sensor.gyro
        if ang_vel_read != None:
            if ang_vel_read[0] != None:
                self.ang_vel[0] = ang_vel_read[0]
            else:
                reset[4] = True
                print("NO ANG VEL 1")
            
            if ang_vel_read[1] != None:
                self.ang_vel[1] = ang_vel_read[1]
            else:
                reset[5] = True
                print("NO ANG VEL 2")
            
            if ang_vel_read[2] != None:
                self.ang_vel[2] = ang_vel_read[2]
            else:
                reset[6] = True
                print("NO ANG VEL 3")
        else:
            print("NO ANG VEL DATA")

        #if ang_vel_read[0]:
        #    self.ang_vel = ang_vel_read
        #else:
        #    print("NO ANG VEL DATA")

        if all(reset):
            print("RESETTING BNO-055")
            print("New heading offset: {}".format(self.backup_heading))
            self.heading_offset = self.backup_heading
            self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

        # Set message values
        msg.heading = self.heading
        msg.lin_acc_x = self.lin_acc[0] + self.sensor.offsets_accelerometer[0] # TODO: Add calibration offset - sensor.offsets_accelerometer[0]
        msg.lin_acc_y = self.lin_acc[1] + self.sensor.offsets_accelerometer[1] # TODO: Add calibration offset - sensor.offsets_accelerometer[1]
        msg.lin_acc_z = self.lin_acc[2] + self.sensor.offsets_accelerometer[2] # TODO: Add calibration offset - sensor.offsets_accelerometer[2]
        msg.ang_vel = self.ang_vel      # TODO: Add calibration offset
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "[H: {}; LA: {}, {}, {}, AV: {}]"'.format(msg.heading, msg.lin_acc_x, msg.lin_acc_y, msg.lin_acc_z, msg.ang_vel))

class UltrasonicPublisher(Node):

    def __init__(self):
        super().__init__('ultasonic_publisher')
        self.publisher_ = self.create_publisher(Ultrasonic, 'ultrasonic', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.L_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D5, echo_pin=board.D6)
        self.R_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.C1, echo_pin=board.C0)
        self.L_dist = 1000.0
        self.R_dist = 1000.0

    def timer_callback(self):
        msg = Ultrasonic()
        # Read sensor values
        try:
            L_dist_read = self.L_sensor.distance
            if L_dist_read:
                self.L_dist = L_dist_read
        except RuntimeError:
            pass

        try:
            R_dist_read = self.R_sensor.distance
            if R_dist_read:
                self.R_dist = R_dist_read
        except RuntimeError:
            pass

        # Set message values
        msg.l_dist_cm = self.L_dist
        msg.r_dist_cm = self.R_dist
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "[Dist: L:{}, R:{}]"'.format(msg.l_dist_cm, msg.r_dist_cm))


def main(args=None):
    rclpy.init(args=args)

    imu_pub = IMUPublisher()
    us_pub = UltrasonicPublisher()

    while True:
        #start = datetime.datetime.now()

        rclpy.spin_once(imu_pub)
        #imu_stop = datetime.datetime.now()

        rclpy.spin_once(us_pub)
        #us_stop = datetime.datetime.now()

        #stop = datetime.datetime.now()
        #print("Split: {} ms".format((stop-start).total_seconds() * 1000))
        #print("\tIMU: {} ms".format((imu_stop - start).total_seconds() * 1000))
        #print("\tUS:  {} ms".format((us_stop - imu_stop).total_seconds() * 1000))

    rclpy.shutdown()