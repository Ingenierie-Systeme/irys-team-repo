import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GNSSViewer(Node):
    """Nœud ROS 2 qui lit /input/gnss et affiche la position."""

    def __init__(self):
        super().__init__('gnss_viewer')
        # Topic paramétrable :
        #   ros2 run sensor_subscriber gnss_viewer --ros-args -p topic:=/input/gnss
        self.declare_parameter('topic', '/input/gnss')
        topic = self.get_parameter('topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            NavSatFix,
            topic,
            self.on_fix,
            qos_profile_sensor_data
        )
        self.subscription  # évite l'avertissement "unused variable"
        self.get_logger().info(f'Lecture du topic : {topic}')

    def on_fix(self, msg: NavSatFix):
        # Les constantes de statut vivent sur NavSatStatus, pas NavSatFix
        status_names = {
            NavSatStatus.STATUS_NO_FIX: 'NO_FIX',
            NavSatStatus.STATUS_FIX: 'FIX',
            NavSatStatus.STATUS_SBAS_FIX: 'SBAS_FIX',
            NavSatStatus.STATUS_GBAS_FIX: 'GBAS_FIX',
        }
        status = status_names.get(msg.status.status, f'UNKNOWN({msg.status.status})')

        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            self.get_logger().warn('GNSS : pas de fix')
            return

        # Ligne lisible façon récepteur GNSS (NMEA-like)
        self.get_logger().info(
            f'--- GNSS ({status}) ---\n'
            f'latitude  : {msg.latitude:10.6f} °\n'
            f'longitude : {msg.longitude:10.6f} °\n'
            f'altitude  : {msg.altitude:8.2f} m\n'
            f'covariance (diag lat/lon/alt) : '
            f'{msg.position_covariance[0]:.2f} / '
            f'{msg.position_covariance[4]:.2f} / '
            f'{msg.position_covariance[8]:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GNSSViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()