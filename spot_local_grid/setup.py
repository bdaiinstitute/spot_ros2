from setuptools import find_packages, setup

package_name = 'spot_local_grid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'bosdyn-api',
        'bosdyn-client',
        'bosdyn-core',
        'bosdyn-mission',
        ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ry.roche@icloud.com',
    description="Adds a ROS2 publisher for Spot's Local Grid",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_grid_pub = spot_local_grid.local_grid_pub:main'
        ],
    },
)
