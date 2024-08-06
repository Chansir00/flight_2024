from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'opencv_tools'

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
    maintainer_email='chan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
      'img_publisher = opencv_tools.basic_image_publisher:main',
      'img_subscriber = opencv_tools.basic_image_subscriber:main',
      'to_img_mcu = opencv_tools.to_img_mcu:main',
      #‘opencv_subscriber = opencv_tools.opencv_rosnode_test:main',
      #'test_shape = opencv_tools.test_shape:main',
      'down_image_subscriber = opencv_tools.down_cam_subscribe:main',
      'down_image_publisher = opencv_tools.down_image_publisher:main',
      #'sticker_ctr=opencv_tools.sticker_ctr:main',
      'circle_ctr=opencv_tools.circle_ctr:main',
      #'imu_publish = opencv_tools.rc_imu:main',
      'left_qr_pub = opencv_tools.left_QR_scan:main',
      'right_qr_pub = opencv_tools.right_QR_scan:main',
    ],
},
)
