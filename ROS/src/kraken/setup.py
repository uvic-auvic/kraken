from setuptools import setup

package_name = 'kraken'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auvic',
    maintainer_email='philipesclamado@uvic.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_sensor = kraken.depth_sensor_node:main',
            'imu = kraken.imu_node:main',
            'peripheral_controller = kraken.peripheral_controller_node:main',
        ],
    },
)
