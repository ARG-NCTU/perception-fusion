import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_pcd_projection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        # Include all config (json) files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.json'))),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='tzuchichen',
    maintainer_email='tzuchichen.en12@nycu.edu.tw',
    description='Camera & Lidar or Radar pointcloud projection package for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_pcd_projection_node = camera_pcd_projection.camera_pcd_projection_node:main',
            'generate_calibration = camera_pcd_projection.generate_calibration:main',
            'camera_merge_node = camera_pcd_projection.camera_merge_node:main',
            'video_saver_node = camera_pcd_projection.video_saver_node:main',
        ],
    },
)
