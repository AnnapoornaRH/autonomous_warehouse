import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_task_allocator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_task_allocator']),
            ('share/warehouse_task_allocator', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Deadline and workload aware task allocation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task_allocator = warehouse_task_allocator.task_allocator:main'
        ],
    },
)
