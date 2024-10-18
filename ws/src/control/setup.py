from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Remove the action files line if no action files are present in this package.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eyad',
    maintainer_email='eyadmaeen1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm = control.arm:main',  # arm set
            'arm2 = control.arm2:main',
            'arm3 = control.arm3:main',
            'autonomous_rov_server = control.autonomous_rov_server:main',
            'autonomous_rov_client = control.autonomous_rov_client:main',
        ],
    },
)

