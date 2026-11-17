import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_metrics'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_metrics']),
            ('share/warehouse_metrics', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Performance metrics and CSV export',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'metrics_node = warehouse_metrics.metrics_node:main'
        ],
    },
)
