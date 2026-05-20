from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'assgn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'Assgn.launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'world.launch.py'))),
        (os.path.join('share', package_name, 'assgn'), glob(os.path.join('assgn', 'tonystarklogic.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', 'rviz.rviz'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', 'IronManCave.sdf'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', 'erc_bot.gazebo'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', 'IronRover.urdf'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', 'materials.xacro'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', 'IronMan_ROS_Body.dae'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', 'IronManWheelLeftFront.dae'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', 'IronManWheelRightFront.dae'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', 'lidar.dae'))),
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnavb',
    maintainer_email='arnavbodigepally@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node = assgn.tonystarklogic:main',
        ],
    },
)
