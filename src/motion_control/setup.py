from setuptools import setup

package_name = 'motion_control'

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
    maintainer='adam',
    maintainer_email='arezza.adam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_controller=motion_control.diff_drive:main',
            'pid_controller=motion_control.PID:main',
            'velocities=motion_control.velocities:main',
            'base_tf_broadcaster=motion_control.base_tf_broadcaster:main',
            'imu_tf_broadcaster=motion_control.imu_tf_broadcaster:main',
            'wheel_odometry=motion_control.wheel_odometry:main'
        ],
    },
)
