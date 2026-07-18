from setuptools import find_packages, setup


package_name = 'cub_simulation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='cub',
    maintainer_email='cub@todo.todo',
    description='Common ROS 2 utilities and integration tests for CubSim.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cub_simulation_smoke = cub_simulation.topic_probe:main',
        ],
    },
)
