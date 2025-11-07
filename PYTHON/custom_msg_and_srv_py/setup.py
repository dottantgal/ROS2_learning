from setuptools import setup

package_name = 'custom_msg_and_srv_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/CapitalFullName.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antonio Mauro Galiano',
    maintainer_email='antoniomauro.galiano@gmail.com',
    description='Capital full name service definition with example server and client nodes.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capital_full_name_server = custom_msg_and_srv_py.capital_full_name_server:main',
            'capital_full_name_client = custom_msg_and_srv_py.capital_full_name_client:main',
        ],
    },
)
