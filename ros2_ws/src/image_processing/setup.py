import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'image_processing'

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
        'numpy'
    ],
    zip_safe=True,
    maintainer='tzuchichen',
    maintainer_email='tzuchichen.en12@nycu.edu.tw',
    description='Image processing package for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = image_processing.rtsp_to_compressed_image:main',
        ],
    },
)
