# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
from setuptools import find_packages, setup

package_name = 'mowgli_airseekers_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/airseekers_bridge.yaml']),
        ('share/' + package_name + '/launch', ['launch/airseekers_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mowgli Project',
    maintainer_email='cedric@mowgli.dev',
    description='Hardware bridge for the Airseekers Tron (HARDWARE_BACKEND=airseekers).',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'airseekers_bridge_node = mowgli_airseekers_bridge.bridge_node:main',
        ],
    },
)
