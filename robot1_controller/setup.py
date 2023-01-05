from setuptools import setup
import os
from glob import glob

package_name = 'robot1_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='depressedcoder',
    maintainer_email='jacobja3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_1_map_processor = ' + package_name + '.mapping:main',
            'robot_1_motion_planner = '+ package_name + '.robot_motion:main'
        ],
    },
)
