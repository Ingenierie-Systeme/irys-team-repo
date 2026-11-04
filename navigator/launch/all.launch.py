from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(package="navigator", output="screen",
            executable="navigator",
            name="navigator",
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
    ])
