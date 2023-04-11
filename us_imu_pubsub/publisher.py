# ROS Imports
import threading
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
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=2000)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.euler = [0.0, 0.0, 0.0]
        self.lin_acc = [0.0, 0.0, 0.0]
        self.ang_vel = [0.0, 0.0, 0.0]

    def timer_callback(self):
        msg = IMU()
        # Read sensor values
        lin_acc_read = self.sensor.linear_acceleration
        if lin_acc_read[0]:
            self.lin_acc = lin_acc_read

        euler_read = self.sensor.euler
        if euler_read[0]:
            self.euler = euler_read

        ang_vel_read = self.sensor.gyro
        if ang_vel_read[0]:
            self.ang_vel = ang_vel_read

        # Set message values
        msg.heading = self.euler[0]
        msg.lin_acc_x = self.lin_acc[0] # TODO: Add calibration offset - sensor.offsets_accelerometer[0]
        msg.lin_acc_y = self.lin_acc[1] # TODO: Add calibration offset - sensor.offsets_accelerometer[1]
        msg.lin_acc_z = self.lin_acc[2] # TODO: Add calibration offset - sensor.offsets_accelerometer[2]
        msg.ang_vel = self.ang_vel      # TODO: Add calibration offset
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "[H: {}; LA: {}, {}, {}, AV: {}]"'.format(msg.heading, msg.lin_acc_x, msg.lin_acc_y, msg.lin_acc_z, msg.ang_vel))

class UltrasonicPublisher(Node):

    def __init__(self):
        super().__init__('ultasonic_publisher')
        self.publisher_ = self.create_publisher(Ultrasonic, 'ultrasonic', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i2c = busio.I2C(board.SCL, board.SDA, frequency=2000)
        self.L_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D5, echo_pin=board.D6)
        self.R_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.C1, echo_pin=board.C0)
        #self.sensor = adafruit_hcsr04.BNO055_I2C(self.i2c)
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

    #pub_exec = Executor()
    #pub_exec.add_node(imu_pub)
    #pub_exec.add_node(us_pub)

    # Spin Ultrasonic node in a separate thread
    # thread = threading.Thread(target=rclpy.spin, args=(us_pub,), daemon=True)
    # thread.start()

    # Spin IMU node in main
    #rclpy.spin(imu_pub)
    while True:
        rclpy.spin_once(imu_pub)
        rclpy.spin_once(us_pub)
    #pub_exec.spin()

    rclpy.shutdown()
    #thread.join()