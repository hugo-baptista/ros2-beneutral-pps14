from setuptools import find_packages, setup

package_name = 'converters'

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
    description='Python package that subscribers the IMU topics and writes the messages in CSV files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor0 = converters.motor0_status:main',
            'gps = converters.gps_receive:main',
            'gpio = converters.can_gpio_key_switch:main',
            'imu = converters.IMU_data:main',
        ],
    },
)
