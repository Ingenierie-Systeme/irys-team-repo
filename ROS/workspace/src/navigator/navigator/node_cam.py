#!/usr/bin/env python3

import time

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image


class CameraNode(Node):
    """Publie les images de la caméra USB sur /input/image."""

    def __init__(self) -> None:
        super().__init__('camera')

        self.declare_parameter(
            'camera_device',
            '/dev/v4l/by-path/'
            'platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-'
            'video-index0',
        )
        self.declare_parameter('frame_rate', 30.0)
        self.camera_device = str(
            self.get_parameter('camera_device').value
        )
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        if self.frame_rate <= 0.0:
            raise ValueError('frame_rate doit être supérieur à zéro.')

        self.bridge = CvBridge()
        self.capture: cv2.VideoCapture | None = None
        self.last_open_attempt = 0.0
        self.publisher = self.create_publisher(
            Image,
            '/input/image',
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(
            1.0 / self.frame_rate,
            self.publish_frame,
        )

        self.get_logger().info(
            f'Caméra {self.camera_device} -> /input/image '
            f'({self.frame_rate:.1f} images/s).'
        )

    def publish_frame(self) -> None:
        if self.capture is None:
            now = time.monotonic()
            if now - self.last_open_attempt < 2.0:
                return

            self.last_open_attempt = now
            self.capture = cv2.VideoCapture(
                self.camera_device,
                cv2.CAP_V4L2,
            )
            if not self.capture.isOpened():
                self.capture.release()
                self.capture = None
                self.get_logger().warning(
                    f'Impossible d’ouvrir la caméra '
                    f'{self.camera_device}; nouvelle tentative dans 2 s.'
                )
                return

            self.get_logger().info(
                f'Caméra {self.camera_device} ouverte.'
            )

        success, frame = self.capture.read()
        if not success or frame is None:
            self.capture.release()
            self.capture = None
            self.get_logger().warning(
                f'Échec de lecture de la caméra {self.camera_device}; '
                'tentative de reconnexion.'
            )
            return

        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(image)

    def destroy_node(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
