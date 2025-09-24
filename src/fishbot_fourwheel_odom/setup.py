from setuptools import setup
import os
from glob import glob

package_name = 'fishbot_fourwheel_odom'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishbot',
    maintainer_email='noreply@example.com',
    description='Four-wheel (left/right pair) differential odometry for Nav2.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fourwheel_diff_odom = fishbot_fourwheel_odom.fourwheel_diff_odom_node:main',
        ],
    },
)
