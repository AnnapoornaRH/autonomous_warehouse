import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_robot_agent'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_robot_agent']),
            ('share/warehouse_robot_agent', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Per-robot state machine (navigation, aisle reservations, pick and place)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_agent = warehouse_robot_agent.robot_agent:main'
        ],
    },
)
