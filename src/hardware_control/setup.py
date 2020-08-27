from setuptools import setup
import os
from glob import glob

package_name = 'hardware_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam',
    maintainer_email='arezza.adam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        #node_name=package_name.node_script:main <--- add the main if there is a main function in script
		'motor_cmds=hardware_control.motor_cmds:main',
        'serial_comms=hardware_control.serial_comms:main',
        'imu_node=hardware_control.imu:main',
        'motor_node=hardware_control.motor_out:main'
        ],
    },
)
