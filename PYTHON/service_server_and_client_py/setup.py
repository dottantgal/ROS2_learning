from setuptools import setup

package_name = 'service_server_and_client_py'

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
    maintainer_email='foo@foo.foo',
    description='Example nodes to create service server and service client',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'service_node = service_server_and_client_py.service_node:main',
            'client_node = service_server_and_client_py.client_node:main',
            'service_node_class = service_server_and_client_py.service_node_class:main',
            'client_node_class = service_server_and_client_py.client_node_class:main',
        ],
    },
)

