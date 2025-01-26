from setuptools import find_packages
from setuptools import setup

setup(
    name='smart_car',
    version='0.0.0',
    packages=find_packages(
        include=('smart_car', 'smart_car.*')),
)
