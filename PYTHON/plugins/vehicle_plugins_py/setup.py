from setuptools import setup

package_name = 'vehicle_plugins_py'

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
    maintainer='Antonio Mauro Galiano',
    maintainer_email='user@todo.todo',
    description='Plugin example',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'create_vehicle = vehicle_plugins_py.create_vehicle:main',
        ],
        # Entry points for plugin discovery
        'vehicle_base_py.plugins': [
            'motorbike = vehicle_plugins_py.vehicle_plugins:create_motorbike',
            'bicycle = vehicle_plugins_py.vehicle_plugins:create_bicycle',
            'truck = vehicle_plugins_py.vehicle_plugins:create_truck',
        ],
    },
)

