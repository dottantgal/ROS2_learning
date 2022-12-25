from setuptools import setup

package_name = 'publisher_and_subscriber_py'

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
    description='Example nodes to understand publisher and subscriber',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher_node = publisher_and_subscriber_py.simple_publisher_node:main',
            'simple_publisher_class_node = publisher_and_subscriber_py.simple_publisher_class_node:main'
        ],
    },
)
