from setuptools import find_packages, setup

package_name = 'py_action'

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
    description='Python action server and client tutorial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fibonacci_action_server = py_action.fibonacci_action_server:main',
            'fibonacci_action_client = py_action.fibonacci_action_client:main',
        ],
    },
)
