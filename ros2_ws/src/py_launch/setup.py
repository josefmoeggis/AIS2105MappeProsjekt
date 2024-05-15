from setuptools import setup, find_packages

package_name = 'py_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'py_launch/launch.py'  # Adjust this line if necessary
        ]),
    ],
    entry_points={
        'console_scripts': [],
    },
    maintainer='joe',
    maintainer_email='josefnm@stud.ntnu.no',
    description='Central launch package for ROS 2 nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
)
