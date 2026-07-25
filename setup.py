from setuptools import setup

package_name = 'meridian_slam'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blu-y',
    maintainer_email='a_o@kakao.com',
    description='SLAM placeholder node publishing per-frame pose estimates.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = meridian_slam.slam_node:main',
        ],
    },
)
