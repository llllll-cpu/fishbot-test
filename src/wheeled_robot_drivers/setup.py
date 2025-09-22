from setuptools import find_packages, setup

package_name = 'wheeled_robot_drivers'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['wheeled_robot_drivers/launch/hda536t_imu.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='fishbot',
    maintainer_email='you@example.com',
    description='HDA536T IMU → sensor_msgs/Imu (ROS 2) for Nav2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hda536t_imu_node = wheeled_robot_drivers.imu_hda536t_node:main',
        ],
    },
)
