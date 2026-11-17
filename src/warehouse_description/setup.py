import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/warehouse_description']),
            ('share/warehouse_description', ['package.xml']),
            ('share/warehouse_description/urdf', [f for f in glob('urdf/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warehouse',
    maintainer_email='warehouse@example.com',
    description='Mobile manipulator xacro/URDF',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            
        ],
    },
)
