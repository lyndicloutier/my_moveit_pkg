# pylint: disable=missing-module-docstring
import os
from glob import glob
from setuptools import setup
# from setuptools import find_packages, setup

PACKAGE_NAME = 'my_moveit_pkg'

setup(
    name=PACKAGE_NAME,
    version='2.35.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', PACKAGE_NAME, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csrobot',
    maintainer_email='lyndi_cloutier@student.uml.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ur5_arm_test = my_moveit_pkg.ur5_arm_test:main",
            "publisher_testing = my_moveit_pkg.publisher_testing:main"
        ],
    },
)
