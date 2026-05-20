from setuptools import find_packages, setup

package_name = 'erc_ros2_simple_arm_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aswath Sahoo',
    maintainer_email='aswath_sahoo@yahoo.com',
    description='Python nodes for simulation of a robotic arm with Gazebo Harmonic and ROS Jazzy for ERC MOGI ROS2 course',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_joint_angles = erc_ros2_simple_arm_py.send_joint_angles:main',
            'inverse_kinematics = erc_ros2_simple_arm_py.inverse_kinematics:main',
            'integrated_pickup = erc_ros2_simple_arm_py.integrated_pickup:main',
        ],
    },
)
