import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class IMUViewer(Node):
    """Nœud ROS 2 qui lit sensor_msgs/msg/Imu et affiche les mesures.
    ros2 run sensor_subscriber imu_viewer --ros-args -p topic:=/input/imu
    """

    def __init__(self):
        super().__init__('imu_viewer')
        # Topic paramétrable :
        #   ros2 run sensor_subscriber imu_viewer --ros-args -p topic:=/input/imu
        self.declare_parameter('topic', '/input/imu')
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # QoS "sensor data" : best effort, adapté aux capteurs temps réel.
        # Important : si le publisher Unity est en default (reliable),
        # un subscriber best_effort recevra quand même les messages,
        # mais l'inverse (publisher best_effort -> subscriber reliable)
        # ne fonctionnerait PAS. En cas de doute, retirer ce paramètre QoS.
        self.subscription = self.create_subscription(
            Imu,
            topic,
            self.on_imu,
            qos_profile_sensor_data
        )
        self.subscription  # évite l'avertissement "unused variable"
        self.get_logger().info(f'Lecture du topic : {topic}')

    def on_imu(self, msg: Imu):
        # Orientation : quaternion -> roulis/tangage/lacet (degrés)
        q = msg.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                          1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))

        self.get_logger().info(
            f'--- IMU ---\n'
            f'orientation (deg) : roll={math.degrees(roll):8.2f} '
            f'pitch={math.degrees(pitch):8.2f} yaw={math.degrees(yaw):8.2f}\n'
            f'gyro (rad/s)      : x={msg.angular_velocity.x:8.3f} '
            f'y={msg.angular_velocity.y:8.3f} z={msg.angular_velocity.z:8.3f}\n'
            f'accel (m/s²)      : x={msg.linear_acceleration.x:8.3f} '
            f'y={msg.linear_acceleration.y:8.3f} z={msg.linear_acceleration.z:8.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = IMUViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()