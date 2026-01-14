from setuptools import setup
from glob import glob
import os

package_name = 'lidar_pose_estimation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UTC ISCF Team',
    maintainer_email='iscf@utc.fr',
    description='6DOF pose estimation using vertical LiDAR and deep learning',
    license='MIT',
    entry_points={
        'console_scripts': [
            'angle_filter_tester = lidar_pose_estimation.angle_filter_tester:main',
            'filter_node = lidar_pose_estimation.filter_node:main',
            'pose_estimator = lidar_pose_estimation.pose_estimator_node:main',
        ],
    },
)
