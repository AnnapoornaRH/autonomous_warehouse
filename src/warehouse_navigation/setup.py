import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_navigation']),
            ('share/warehouse_navigation', ['package.xml']),
            ('share/warehouse_navigation/launch', [f for f in glob('launch/*') if os.path.isfile(f)]),
            ('share/warehouse_navigation/config', [f for f in glob('config/*') if os.path.isfile(f)]),
            ('share/warehouse_navigation/maps', [f for f in glob('maps/*') if os.path.isfile(f)]),
            ('share/warehouse_navigation/behavior_trees', [f for f in glob('behavior_trees/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Multi-robot Nav2 configuration, velocity gate',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_gate = warehouse_navigation.cmd_vel_gate:main'
        ],
    },
)
