from setuptools import setup
import os
from glob import glob

package_name = 'fishbot_nav'

def data_files_in(dest, pattern):
    return [(dest, glob(pattern))]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + data_files_in('share/' + package_name + '/launch', 'launch/*.launch.py') \
      + data_files_in('share/' + package_name + '/config', 'config/*.yaml') \
      + data_files_in('share/' + package_name + '/map', 'map/*.*') \
      + data_files_in('share/' + package_name + '/urdf', 'urdf/*.*'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishbot',
    maintainer_email='you@example.com',
    description='Nav2 bringup for Fishbot with lidar+imu+odom+ultrasonic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
