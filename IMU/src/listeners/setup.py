from setuptools import find_packages, setup

package_name = 'listeners'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugobaptista',
    maintainer_email='pg50416@alunos.uminho.pt',
    description='Python package with subscribers for the IMU topics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor0 = listeners.motor0_status:main',
            'gps = listeners.gps_receive:main',
            'gpio = listeners.can_gpio_key_switch:main',
            'imu = listeners.IMU_data:main',
        ],
    },
)
