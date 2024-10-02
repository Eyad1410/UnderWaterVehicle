from setuptools import setup, find_packages

package_name = 'my_arm_service'

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
    maintainer='eyad',
    maintainer_email='eyadmaeen1@gmail.com',
    description='Package for controlling arm and vehicle movement',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'arm_client = my_arm_service.arm_client:main',
            'move_client = my_arm_service.move_client:main',
        ],
    },
)

