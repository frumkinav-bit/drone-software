from setuptools import find_packages, setup

package_name = 'uav_lidar'

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
    description='Обработка данных лидара БПЛА',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_processor = uav_lidar.lidar_processor:main',
            'lidar_scan_player = uav_lidar.lidar_scan_player:main',
        ],
    },
)
