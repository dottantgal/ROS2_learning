from setuptools import setup
import os
from glob import glob

package_name = 'dynamic_tf2_publisher_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='white',
    maintainer_email='antoniomauro.galiano@gmail.com',
    description='TF2 publisher with params reconfiguration at run time',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tf2_publisher_node = dynamic_tf2_publisher_py.tf2_publisher_node:main',
        ],
    },
)

