from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    return LaunchDescription([
        Node(package="navigator", output="screen",
            executable="navigator",
            name="navigator",
        ),
        Node(package="navigator", output="screen",
            executable="camera",
            name="camera",
        ),
        Node(package="navigator", output="screen",
            executable="rc",
            name="rc",
        ),
        Node(package="navigator", output="screen",
            executable="baro",
            name="baro",
        ),
        Node(package="navigator", output="screen",
            executable="fusion",
            name="fusion",
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
            'start_state_estimator',
            default_value='true',
            description='Démarrer l’estimateur d’état du robot.',
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
            package='robot_pose',
            executable='state_estimator',
            name='robot_state_estimator',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_state_estimator')
            ),
        ),
    ])
