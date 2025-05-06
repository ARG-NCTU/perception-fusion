import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bag_processing'

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
    ],
    zip_safe=True,
    maintainer='tzuchichen',
    maintainer_email='tzuchichen.en12@nycu.edu.tw',
    description='Image processing package for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'filter_ros2_bag = bag_processing.filter_ros2_bag:main',
            'fix_time = bag_processing.fix_time:main',
            'save_perceptions = bag_processing.save_perceptions:main',
            'relay_tf = bag_processing.relay_tf:main'
        ],
    },
)
