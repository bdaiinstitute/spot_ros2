from setuptools import find_packages
from setuptools import setup

setup(
    name='spot_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('spot_msgs', 'spot_msgs.*')),
)
