from setuptools import setup
import os 
from glob import glob

package_name = 'robot_world_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'worlds/'), glob('./worlds/*')),

        # Path to the warehouse sdf file
        #(os.path.join('share', package_name,'models/my_robot1/'), glob('./models/my_robot1/*')),
        #(os.path.join('share', package_name,'models/my_robot2/'), glob('./models/my_robot2/*')),
        (os.path.join('share', package_name,'models/my_robot/'), glob('./models/my_robot/model.urdf')),
        (os.path.join('share', package_name,'models/my_robot/'), glob('./models/my_robot/model.sdf')),
        (os.path.join('share', package_name,'models/my_robot/'), glob('./models/my_robot/model.config')),
        (os.path.join('share', package_name,'models/my_robot/meshes'), glob('./models/my_robot/meshes/*')),


        # Path to the mobile robot sdf file
        #(os.path.join('share', package_name,'models/mobile_warehouse_robot/'), glob('./models/mobile_warehouse_robot/*')),
        
        # Path to the world file (i.e. warehouse + global environment)
        (os.path.join('share', package_name,'models/'), glob('./worlds/*')),
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
            'spawn_robot = ' + package_name + '.spawn_robot:main'
        ],
    },
)
