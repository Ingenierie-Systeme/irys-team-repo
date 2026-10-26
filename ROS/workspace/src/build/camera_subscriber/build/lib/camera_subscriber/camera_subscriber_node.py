'''Noeud lisant le topic camer/image 
et affiche l'image'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    """Nœud ROS 2 qui lit /camera/image et affiche l'image en direct."""

    def __init__(self):
        super().__init__('camera_viewer')
        # Déclare un paramètre pour le topic, modifiable en ligne de commande :
        #   ros2 run <pkg> camera_viewer --ros-args -p topic:=/unity/camera/image_raw
        self.declare_parameter('topic', '/camera/image')
        topic = self.get_parameter('topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.on_image,
            10  # taille de la queue (QoS history depth)
        )
        self.subscription  # évite l'avertissement "unused variable"
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