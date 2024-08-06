from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'tf2_flight'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chan',
    maintainer_email='a642717304@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_frame_publisher = tf2_flight.world_frame_publisher:main',
            'offset_calculator = tf2_flight.offset_calculator:main',
            'pose_pub=tf2_flight.init_pose:main',
            'tf_listener=tf2_flight.tf_listener:main',
            'main3=tf2_flight.test1:main',
        ],
    },
)
