#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField

import imufusion
import numpy as np

from math import pi

from navigator.config import *


class FusionNode(Node):

    def __init__(self):

        super().__init__("fusion")

        ### Fusion parameters
        self.sample_rate = FREQ_IMU
        self.ahrs = imufusion.Ahrs()
        # self.offset = imufusion.Offset(self.sample_rate)
        self.offset = imufusion.Bias()
        self.offset.set_settings(imufusion.BiasSettings(sample_rate=self.sample_rate))
        self.ahrs.set_settings(imufusion.AhrsSettings(
            sample_rate=self.sample_rate,
            convention=imufusion.CONVENTION_NED,
            gain=0.5,
            gyroscope_range=2000,
            acceleration_rejection=10,
            magnetic_rejection=10,
            rejection_timeout=5,)
        )

        ### Calibration
        self.mag_hard = np.array([-3.28499185, 5.48995909,-0.22732013])
        self.mag_soft = np.eye(3)
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        self.accel_misal = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]])
        self.gyro_offset = np.array([
            -0.01308278,
             0.04102378,
             0.0631724])

        ### State
        self.latest_imu = None
        self.latest_mag = None
        self.last_time = self.get_clock().now()

        ### Subscribers
        self.create_subscription(
            Imu,
            "/input/imu",
            self.imu_callback,
            10)
        self.create_subscription(
            MagneticField,
            "/input/mag",
            self.mag_callback,
            10)

        ### Publisher
        self.attitude_pub = self.create_publisher(
            Imu,
            "/state/attitude",
            10)
        
        self.get_logger().info("Fusion node started.")

    ### IMU callback
    def imu_callback(self, msg):
        self.latest_imu = msg
        self.update_fusion()

    ### Magnetometer callback
    def mag_callback(self, msg):
        self.latest_mag = msg

    ### Fusion
    def update_fusion(self):
        if self.latest_imu is None: return
        if self.latest_mag is None: return

        imu_msg = self.latest_imu
        mag_msg = self.latest_mag

        ### Raw measurements
        accel = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z])
        gyro = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z])
        mag = np.array([
            mag_msg.magnetic_field.x,
            mag_msg.magnetic_field.y,
            mag_msg.magnetic_field.z])

        ### Calibration
        accel = ((1.0/9.81) * self.accel_misal.dot(accel - self.accel_offset))
        gyro = ((1.0/4.0)*(180.0/pi) * (gyro - self.gyro_offset))
        gyro = self.offset.update(gyro)
        mag = self.mag_soft.dot(mag - self.mag_hard)

        ### Delta time
        now = self.get_clock().now()
        delta_time = (
            now - self.last_time
        ).nanoseconds * 1e-9
        self.last_time = now
        if delta_time <= 0.0:
            return

        ### AHRS update
        self.ahrs.set_sample_period(delta_time)
        self.ahrs.update(
            gyro,
            accel,
            mag,
        )

        ### Publish attitude
        self.publish_attitude()

    def publish_attitude(self):
        msg = Imu()
        msg.header.stamp = (self.get_clock().now().to_msg())

        quaternion = self.ahrs.get_quaternion()
        msg.orientation.w = float(quaternion[0])
        msg.orientation.x = float(quaternion[1])
        msg.orientation.y = float(quaternion[2])
        msg.orientation.z = float(quaternion[3])

        self.attitude_pub.publish(msg)


def main():

    rclpy.init()

    node = FusionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()