from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'tb3_multi_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models', 'turtlebot3_waffle'), glob(os.path.join('models', 'turtlebot3_waffle', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itsmevjnk',
    maintainer_email='ngtv0404@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cleanup_node = tb3_multi_launch.cleanup_node:main',
            'delete_watch_node = tb3_multi_launch.delete_watch_node:main',
            'pose_node = tb3_multi_launch.pose_node:main'
        ],
    },
)
