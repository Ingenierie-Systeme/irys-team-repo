from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tcp_ip = LaunchConfiguration('tcp_ip')
    tcp_port = LaunchConfiguration('tcp_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tcp_ip',
            default_value='0.0.0.0',
            description='Adresse d’écoute TCP de ROS-TCP-Endpoint.',
        ),
        DeclareLaunchArgument(
            'tcp_port',
            default_value='10000',
            description='Port d’écoute TCP de ROS-TCP-Endpoint.',
        ),
        DeclareLaunchArgument(
            'start_imu_viewer',
            default_value='true',
            description='Démarrer le viewer du topic /imu/data.',
        ),
        DeclareLaunchArgument(
            'start_gnss_viewer',
            default_value='true',
            description='Démarrer le viewer du topic /gnss/fix.',
        ),
        DeclareLaunchArgument(
            'start_camera_viewer',
            default_value='true',
            description='Démarrer le viewer caméra sur /camera/image.',
        ),
        DeclareLaunchArgument(
            'start_obstacle_monitor',
            default_value='true',
            description='Démarrer le moniteur TTC par flux optique.',
        ),
        DeclareLaunchArgument(
            'start_camera_avoidance',
            default_value='true',
            description='Démarrer le choix de direction par caméra.',
        ),
        DeclareLaunchArgument(
            'start_servo_controller',
            default_value='true',
            description='Démarrer la conversion de direction en commandes servo.',
        ),
        DeclareLaunchArgument(
            'start_state_estimator',
            default_value='true',
            description='Démarrer l’estimateur d’état du robot.',
        ),
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity_endpoint',
            output='screen',
            parameters=[{
                'ROS_IP': tcp_ip,
                'ROS_TCP_PORT': ParameterValue(tcp_port, value_type=int),
            }],
        ),
        Node(
            package='sensor_subscriber',
            executable='imu_viewer',
            name='imu_viewer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_imu_viewer')),
        ),
        Node(
            package='sensor_subscriber',
            executable='gnss_viewer',
            name='gnss_viewer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_gnss_viewer')),
        ),
        Node(
            package='sensor_subscriber',
            executable='camera_viewer',
            name='camera_viewer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_camera_viewer')),
        ),
        Node(
            package='robot_brain',
            executable='optical_flow_emergency_stop',
            name='optical_flow_emergency_stop',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_obstacle_monitor')
            ),
        ),
        Node(
            package='robot_brain',
            executable='camera_obstacle_avoidance',
            name='camera_obstacle_avoidance',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_camera_avoidance')
            ),
        ),
        Node(
            package='robot_control',
            executable='servo_command_publisher',
            name='servo_command_publisher',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_servo_controller')
            ),
        ),
        Node(
            package='robot_pose',
            executable='state_estimator',
            name='robot_state_estimator',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_state_estimator')
            ),
        ),
    ])
