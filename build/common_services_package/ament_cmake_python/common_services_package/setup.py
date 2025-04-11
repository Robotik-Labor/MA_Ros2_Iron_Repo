from setuptools import find_packages
from setuptools import setup

setup(
    name='common_services_package',
    version='0.0.0',
    packages=find_packages(
        include=('common_services_package', 'common_services_package.*')),
)
