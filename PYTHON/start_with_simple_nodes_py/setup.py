from setuptools import setup

package_name = 'start_with_simple_nodes_py'

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
    maintainer_email='tonio@todo.todo',
    description='First ROS2 nodes examples',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'my_first_node = start_with_simple_nodes_py.my_first_node:main',
            'node_with_class = start_with_simple_nodes_py.node_with_class:main',
            'node_timer_without_class = start_with_simple_nodes_py.node_timer_without_class:main',
            'node_timer_with_class = start_with_simple_nodes_py.node_timer_with_class:main',
        ],
    },
)
