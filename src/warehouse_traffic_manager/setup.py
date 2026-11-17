import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_traffic_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_traffic_manager']),
            ('share/warehouse_traffic_manager', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Aisle reservation, queueing, deadlock prevention and recovery',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traffic_manager = warehouse_traffic_manager.traffic_manager:main'
        ],
    },
)
