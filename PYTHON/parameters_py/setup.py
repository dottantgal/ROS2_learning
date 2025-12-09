from setuptools import setup

package_name = 'parameters_py'

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
    description='Example node to handle parameters',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'parameters_node = parameters_py.parameters_node:main',
        ],
    },
)

