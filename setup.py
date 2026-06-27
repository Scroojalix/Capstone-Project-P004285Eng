import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gazebo_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Owen',
    maintainer_email='scroojalixyt@gmail.com',
    description='Simulation on the integration of shared obstacle mapping into WHCA',
    license='MIT License',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_test_node = gazebo_test.test_node:main',
            'mapf_node = gazebo_test.mapf:main',
        ],
    }
)
