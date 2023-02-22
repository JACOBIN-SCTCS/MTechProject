from setuptools import setup
import os
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'robot_controller'), glob('robot_controller/*.py')),

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
            'topological_explore = ' + package_name + '.robot_h_signature:main',
        ],
    },
)
