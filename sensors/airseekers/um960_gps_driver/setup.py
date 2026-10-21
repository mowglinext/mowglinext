from setuptools import find_packages, setup

package_name = 'um960_gps_driver'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
        ('share/' + package_name + '/config', ['config/um960.yaml', 'config/um960_secrets.example.yaml']),
        ('share/' + package_name + '/launch', ['launch/um960.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cedric',
    maintainer_email='cedric@mowgli.dev',
    description=(
        'ROS 2 driver for the Unicore UM960 GNSS/RTK receiver '
        '(NMEA + Unicore ASCII + Unicore binary BESTNAV).'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'um960_node = um960_gps_driver.um960_node:main',
            'fake_ntrip_caster = um960_gps_driver.fake_caster:main',
        ],
    },
)
