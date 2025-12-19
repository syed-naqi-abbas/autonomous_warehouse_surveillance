from setuptools import setup
import os
from glob import glob

package_name = 'warehouse_scanning'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Recognize package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='QR code detection node using OpenCV and PyZbar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Executable name       module:file.function
            'qr_detection = warehouse_scanning.qr_detection:main',
        ],
    },
)
