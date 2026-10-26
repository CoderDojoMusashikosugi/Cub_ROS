from glob import glob
from setuptools import setup

setup(
    name='cub_simulation', version='0.1.0', packages=['cub_simulation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cub_simulation']),
        ('share/cub_simulation', ['package.xml']),
        ('share/cub_simulation/launch', glob('launch/*.launch.py')),
        ('share/cub_simulation/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='cub', maintainer_email='cub@todo.todo',
    description='Cub simulation launch files and velocity-command interface', license='Apache-2.0',
)
