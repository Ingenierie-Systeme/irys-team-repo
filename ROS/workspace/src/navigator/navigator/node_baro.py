#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import navigator.lib_ms5837 as ms5837


class BarometerNode(Node):

    def __init__(self):

        super().__init__("barometer")

        ### Sensor
        self.sensor = ms5837.MS5837_30BA(6)
        if not self.sensor.init():
            raise RuntimeError("MS5837 could not be initialized")
        self.sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)

        ### Publishers
        self.depth_pub = self.create_publisher(
            Float32,
            "/input/depth",
            10,
        )

        ### Timer
        self.timer = self.create_timer(
            0.1,        ### 10 Hz
            self.read_sensor,
        )

        self.get_logger().info("MS5837 initialized.")

    ##########################################################

    def read_sensor(self):
        try:
            if not self.sensor.read():
                return

            msg = Float32()
            msg.data = self.sensor.depth()
            self.depth_pub.publish(msg)

        except Exception as e:

            self.get_logger().warn(str(e))


def main():

    rclpy.init()

    node = BarometerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()