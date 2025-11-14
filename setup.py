from setuptools import find_packages, setup
import os
import glob

package_name = 'pycon_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyqt5'],
    zip_safe=True,
    maintainer='cho',
    maintainer_email='samcho1588@gmail.com',
    description='Python ROS2 package for controlling turtlesim with PyQt GUI',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'con_publisher = pycon_package.con_publisher:main',
            'con_subscriber = pycon_package.con_subscriber:main',
            'pyqt_turtle = pycon_package.pyqt_turtle:main',
        ],
    },
)
