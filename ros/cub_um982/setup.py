import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cub_um982'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='cub',
    maintainer_email='cub@example.com',
    description='ROS2 driver node for Unicore UM982 GNSS receiver with dual-antenna heading and NTRIP RTK',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'um982_node = cub_um982.um982_node:main',
            'configure_um982 = cub_um982.configure_um982:main',
        ],
    },
)
