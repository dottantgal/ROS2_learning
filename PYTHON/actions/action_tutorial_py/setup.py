from setuptools import setup

package_name = 'action_tutorial_py'

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
    description='Action based tutorial',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'simple_action_server = action_tutorial_py.simple_action_server:main',
            'simple_action_client = action_tutorial_py.simple_action_client:main',
            'class_action_server = action_tutorial_py.class_action_server:main',
            'class_action_client = action_tutorial_py.class_action_client:main',
        ],
    },
)

