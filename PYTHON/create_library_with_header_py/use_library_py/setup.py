from setuptools import setup

package_name = 'use_library_py'

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
    maintainer_email='antoniomauro.galiano@gmail.com',
    description='A basic node to use the publisher library',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'use_library = use_library_py.use_library:main',
        ],
    },
)

