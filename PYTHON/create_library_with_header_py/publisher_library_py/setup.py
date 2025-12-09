from setuptools import setup

package_name = 'publisher_library_py'

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
    description='A basic publisher library made with Python module',
    license='TODO: License declaration',
)

