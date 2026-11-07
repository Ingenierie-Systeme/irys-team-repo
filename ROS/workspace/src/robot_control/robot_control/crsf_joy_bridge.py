"""Publish TBS Crossfire RC channels as a ROS Joy message."""

import math

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Joy

from robot_control.crsf_protocol import CrsfFrameParser


class CrsfJoyBridge(Node):
    """Read CRSF frames from a UART and publish normalized RC channels."""

    def __init__(self) -> None:
        super().__init__('crsf_joy_bridge')
        self.declare_parameter('serial_device', '/dev/serial0')
        self.declare_parameter('baudrate', 420000)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('read_rate_hz', 100.0)

        self.serial_device = str(
            self.get_parameter('serial_device').value
        )
        self.baudrate = int(self.get_parameter('baudrate').value)
        read_rate_hz = float(self.get_parameter('read_rate_hz').value)
        if self.baudrate <= 0:
            raise ValueError('baudrate doit être supérieur à zéro.')
        if not math.isfinite(read_rate_hz) or read_rate_hz <= 0.0:
            raise ValueError('read_rate_hz doit être supérieur à zéro.')

        joy_topic = str(self.get_parameter('joy_topic').value)
        self.serial_port = serial.Serial(
            port=self.serial_device,
            baudrate=self.baudrate,
            timeout=0,
        )
        self.parser = CrsfFrameParser()
        self.publisher = self.create_publisher(Joy, joy_topic, 10)
        self.timer = self.create_timer(
            1.0 / read_rate_hz,
            self.read_serial,
        )
        self.get_logger().info(
            f'Lecture CRSF sur {self.serial_device} à '
            f'{self.baudrate} bauds ; '
            f'publication sur {joy_topic}.'
        )

    def read_serial(self) -> None:
        bytes_available = self.serial_port.in_waiting
        if bytes_available == 0:
            return

        data = self.serial_port.read(bytes_available)
        for channels in self.parser.feed(data):
            message = Joy()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'crsf_receiver'
            message.axes = list(channels)
            message.buttons = []
            self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: CrsfJoyBridge | None = None
    try:
        node = CrsfJoyBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.serial_port.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
