import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_order_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_order_manager']),
            ('share/warehouse_order_manager', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Autonomous order generator and order book',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'order_manager = warehouse_order_manager.order_manager:main'
        ],
    },
)
