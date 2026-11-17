import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_bringup']),
            ('share/warehouse_bringup', ['package.xml']),
            ('share/warehouse_bringup/launch', [f for f in glob('launch/*') if os.path.isfile(f)]),
            ('share/warehouse_bringup/config', [f for f in glob('config/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Top level launch files and start-up sequencer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'system_monitor = warehouse_bringup.system_monitor:main'
        ],
    },
)
