import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'whca_rviz_baseline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Owen',
    maintainer_email='scroojalixyt@gmail.com',
    description='WHCA* Experiment Node — Silver 2005 replication with RViz visualization',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'whca_rviz_baseline_node = whca_rviz_baseline.whca_rviz_baseline_node:main'
        ],
    },
)
