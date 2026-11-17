import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_manipulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_manipulation']),
            ('share/warehouse_manipulation', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Arm controller: IK based pick and place action server',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_controller = warehouse_manipulation.arm_controller:main'
        ],
    },
)
