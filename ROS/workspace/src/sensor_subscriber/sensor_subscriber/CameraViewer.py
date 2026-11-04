'''Noeud lisant le topic input/camera et affiche l'image:

ros2 run sensor_subscriber camera_viewer --ros-args -p topic:=/input/camera
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    """Nœud ROS 2 qui lit /input/camera et affiche l'image en direct."""

    def __init__(self):
        super().__init__('camera_viewer')

        self.declare_parameter('topic', '/input/camera')
        topic = self.get_parameter('topic').value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.on_image,
            qos_profile_sensor_data
        )
        self.get_logger().info(f'Affichage du topic : {topic}')

    def on_image(self, msg: Image):
        try:
            # Conversion en image OpenCV (BGR8 pour cv2.imshow)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Erreur de conversion : {e}')
            return

        cv2.imshow('ROS 2 Camera Viewer', cv_image)
        # Nécessaire pour que la fenêtre se rafraîchisse et gère les évènements
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()