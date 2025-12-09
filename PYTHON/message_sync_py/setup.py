from setuptools import setup

package_name = 'message_sync_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antonio Mauro Galiano',
    maintainer_email='dottantgal@users.noreply.github.com',
    description='A ROS2 example package to synchronize messages from topics',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'message_sync = message_sync_py.message_sync:main',
        ],
    },
)

