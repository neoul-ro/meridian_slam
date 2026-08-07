import os
from glob import glob

from setuptools import setup

package_name = 'meridian_slam_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neworin',
    maintainer_email='rlawndud119@gmail.com',
    description='Bringup for FAST-LIVO2 with VLP-16 + VN-100S + D435',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_tf_relay = meridian_slam_bringup.odom_tf_relay:main',
        ],
    },
)
