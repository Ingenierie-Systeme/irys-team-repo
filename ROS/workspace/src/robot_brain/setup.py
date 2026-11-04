from setuptools import find_packages, setup

package_name = 'robot_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noecrrr',
    maintainer_email='noe.carrayrou@ensta.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'optical_flow_emergency_stop = '
            'robot_brain.optical_flow_emergency_stop:main',
            'camera_obstacle_avoidance = '
            'robot_brain.camera_obstacle_avoidance:main',
            'robot_brain = robot_brain.robot_brain:main',
        ],
    },
)
