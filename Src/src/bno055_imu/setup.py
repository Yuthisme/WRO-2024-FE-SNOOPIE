from setuptools import find_packages, setup

package_name = 'bno055_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aupp',
    maintainer_email='reaksaddr4@gmail.com',
    description='Package for publishing IMU data from BNO055 sensor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055= bno055_imu.bno055:main',
        ],
    },
)
