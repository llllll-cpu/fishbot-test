from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fishbot_nav_verification'

def data_files_in(dest, pattern):
    return [(dest, glob(pattern))]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + data_files_in('share/' + package_name + '/launch', 'launch/*.launch.py'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='lee123lii123@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_laser_publisher = fishbot_nav_verification.fake_laser_publisher:main',
            'fake_imu_publisher = fishbot_nav_verification.fake_imu_publisher:main',
            'fake_odom_publisher = fishbot_nav_verification.fake_odom_publisher:main',
            'fake_ultrasonic_publisher = fishbot_nav_verification.fake_ultrasonic_publisher:main',
            'cmd_vel_logger = fishbot_nav_verification.cmd_vel_logger:main',
        ],
    },
)
