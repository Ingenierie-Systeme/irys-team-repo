from glob import glob
import os

from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noeca',
    maintainer_email='noe.carrayrou@ensta.fr',
    description='Launch files to start the robot ROS 2 stack.',
    license='Apache-2.0',
)
