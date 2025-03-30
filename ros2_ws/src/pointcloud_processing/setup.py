import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pointcloud_processing'

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
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'pcl',
    ],
    zip_safe=True,
    maintainer='tzuchichen',
    maintainer_email='tzuchichen.en12@nycu.edu.tw',
    description='Point cloud processing package for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_pose_publisher = pointcloud_processing.fake_pose_publisher:main',
            'dynamic_broadcaster = pointcloud_processing.dynamic_broadcaster:main',
            'transform_pointcloud = pointcloud_processing.transform_pointcloud:main',
            'transform_laserscan = pointcloud_processing.transform_laserscan:main',
            'pointcloud_to_image = pointcloud_processing.pointcloud_to_image:main',
            'pointcloud_to_image_gpu = pointcloud_processing.pointcloud_to_image_gpu:main',
            'save_pointcloud = pointcloud_processing.save_pointcloud:main',
            'laserscan_qos = pointcloud_processing.laserscan_qos:main',
        ],
    },
)
