import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_dashboard_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_dashboard_bridge']),
            ('share/warehouse_dashboard_bridge', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='ROS 2 to web dashboard bridge (FastAPI + WebSocket)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dashboard_bridge = warehouse_dashboard_bridge.bridge_node:main'
        ],
    },
)
