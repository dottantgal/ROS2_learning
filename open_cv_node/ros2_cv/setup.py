from setuptools import setup

package_name = 'ros2_cv'

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
    maintainer='magnum',
    maintainer_email='ogunsdavis53@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'face_detection=ros2_cv.face_detection:main',
        'integration=ros2_cv.open_cv_integration:main',

        ],
    },
)
