import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_simulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_simulation']),
            ('share/warehouse_simulation', ['package.xml']),
            ('share/warehouse_simulation/launch', [f for f in glob('launch/*') if os.path.isfile(f)]),
            ('share/warehouse_simulation/worlds', [f for f in glob('worlds/*') if os.path.isfile(f)]),
            ('share/warehouse_simulation/config', [f for f in glob('config/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Gazebo world, ros_gz bridges, simulation services (ground truth, collisions, items, sim control)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_manager = warehouse_simulation.sim_manager:main',
            'sim_relay = warehouse_simulation.sim_relay:main'
        ],
    },
)
