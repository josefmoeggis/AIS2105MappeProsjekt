from setuptools import find_packages, setup

package_name = 'py_kinematics'

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
    maintainer='joe',
    maintainer_email='josefnm@stud.ntnu.no',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
entry_points={
    'console_scripts': [
        'kinematic_control = py_kinematics.kinematic_control:main',
    ],
}
)
