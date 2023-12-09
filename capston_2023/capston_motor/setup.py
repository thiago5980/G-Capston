import os
from glob import glob
from setuptools import find_packages
from setuptools import setup
package_name = 'capston_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thiago',
    maintainer_email='st9051@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'capston_mobile_motor_node = capston_motor.capston_motor_node:main',
        ],
    },
)
