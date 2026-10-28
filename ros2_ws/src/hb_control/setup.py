from glob import glob

from setuptools import find_packages, setup

package_name = 'hb_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Athena workshop',
    maintainer_email='nssamyakvarma@gmail.com',
    description='Kiwi drive kinematics, odometry and ESP32 serial bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kiwi_drive = hb_control.kiwi_drive_node:main',
            'esp32_bridge = hb_control.esp32_bridge_node:main',
            'wheel_test = hb_control.wheel_test:main',
        ],
    },
)
