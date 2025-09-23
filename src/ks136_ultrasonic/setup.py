from setuptools import setup

package_name = 'ks136_ultrasonic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ks136_ultrasonic.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Publish KS136 front/rear ultrasonic as sensor_msgs/Range for Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ks136_ultrasonic_node = ks136_ultrasonic.node:main',
        ],
    },
)
