from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():


    return LaunchDescription([
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
            'start_robot_brain',
            default_value='true',
            description=(
                'Démarrer la supervision et l’arbitrage des consignes.'
            ),
        ),
        DeclareLaunchArgument(
            'start_servo_controller',
            default_value='true',
            description=(
                'Démarrer la conversion de direction en commandes servo.'
            ),
        ),
        DeclareLaunchArgument(
            'start_crsf_receiver',
            default_value='false',
            description='Lire le récepteur TBS Crossfire en CRSF série.',
        ),
        DeclareLaunchArgument(
            'crsf_serial_device',
            default_value='/dev/serial0',
            description='Port série utilisé par le récepteur CRSF.',
        ),
        DeclareLaunchArgument(
            'start_state_estimator',
            default_value='true',
            description='Démarrer l’estimateur d’état du robot.',
        ),
        DeclareLaunchArgument(
            'start_imu_fusion',
            default_value='true',
            description='Démarrer la fusion IMU/magnétomètre.',
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
            package='robot_brain',
            executable='robot_brain',
            name='robot_brain',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_robot_brain')
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
            package='robot_control',
            executable='crsf_joy_bridge',
            name='crsf_joy_bridge',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_crsf_receiver')
            ),
            parameters=[{
                'serial_device': LaunchConfiguration('crsf_serial_device'),
            }],
        ),
        Node(
            package='navigator',
            executable='fusion',
            name='imu_fusion',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_imu_fusion')
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
