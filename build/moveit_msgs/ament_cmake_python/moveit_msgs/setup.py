from setuptools import find_packages
from setuptools import setup

setup(
    name='moveit_msgs',
    version='2.6.0',
    packages=find_packages(
        include=('moveit_msgs', 'moveit_msgs.*')),
)
