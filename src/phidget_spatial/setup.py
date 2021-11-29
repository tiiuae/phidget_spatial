from setuptools import setup
import os
from glob import glob

package_name = 'phidget_spatial'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('config', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kide Vuojärvi',
    maintainer_email='kide.vuojarvi@unikie.com',
    description='ROS2 node for Phidget Spatial Precision sensors',
    license='BDS',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phidget_spatial_node = phidget_spatial.phidget_spatial:main'
        ],
    },
)
