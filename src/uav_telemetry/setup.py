from setuptools import find_packages, setup

package_name = 'uav_telemetry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vasilisa Frumkina',
    maintainer_email='frumkinav-bit@users.noreply.github.com',
    description='Сбор и запись телеметрии БПЛА',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'telemetry_node = uav_telemetry.telemetry_node:main',
            'telemetry_logger = uav_telemetry.telemetry_logger:main',
        ],
    },
)
