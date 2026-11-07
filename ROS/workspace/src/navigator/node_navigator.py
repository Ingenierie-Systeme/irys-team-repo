#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray

import bluerobotics_navigator as navigator
import numpy as np
from math import pi
import time

from navigator.config import *


class NavigatorNode(Node):

    def __init__(self):
        super().__init__("navigator")

        ### Navigator
        navigator.set_rgb_led_strip_size(1)
        navigator.set_navigator_version(navigator.NavigatorVersion.Version1)
        navigator.set_raspberry_pi_version(navigator.Raspberry.Pi4)
        navigator.init()
        navigator.set_pwm_freq_hz(50)
        navigator.set_pwm_enable(True)

        ### Variables
        self.old_freq = 0
        self.old_phase = 0

        ### Publishers
        self.pub_imu = self.create_publisher(
            Imu,
            "/input/imu",
            10,)
        self.pub_mag = self.create_publisher(
            MagneticField,
            "/input/mag",
            10,)

        ### Subscriber PWM
        self.create_subscription(
            Float32MultiArray,
            "/output/servos",
            self.pwm_callback,
            10,)

        ### Timer IMU
        self.timer = self.create_timer(
            1/FREQ_IMU,      # 100 Hz
            self.read_imu,)

    def read_imu(self):
        accel = navigator.read_accel()
        gyro = navigator.read_gyro()
        mag = navigator.read_mag()

        imu = Imu()

        imu.linear_acceleration.x = accel.x
        imu.linear_acceleration.y = accel.y
        imu.linear_acceleration.z = accel.z

        imu.angular_velocity.x = gyro.x
        imu.angular_velocity.y = gyro.y
        imu.angular_velocity.z = gyro.z

        self.pub_imu.publish(imu)

        mag_msg = MagneticField()

        mag_msg.magnetic_field.x = -mag.y
        mag_msg.magnetic_field.y = mag.x
        mag_msg.magnetic_field.z = mag.z

        self.pub_mag.publish(mag_msg)

    def pwm_callback(self, msg):
        servo1 = msg.data[0]        # between -1 and 1
        servo2 = msg.data[1]        # between -1 and 1

        servo1 = np.clip(servo1, -1, 1)
        servo2 = np.clip(servo2, -1, 1)

        duty_servo1 = DUTY_MID + servo1 * (DUTY_MAX-DUTY_MID)
        duty_servo2 = DUTY_MID + servo2 * (DUTY_MAX-DUTY_MID)

        navigator.set_pwm_channel_duty_cycle(1, duty_servo1)
        navigator.set_pwm_channel_duty_cycle(0, duty_servo2)


def main():

    rclpy.init()

    node = NavigatorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
