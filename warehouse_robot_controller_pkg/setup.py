import os
from setuptools import setup
from glob import glob

package_name = 'warehouse_robot_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dreampc2',
    maintainer_email='lward@umes.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'robot_controller = warehouse_robot_controller_pkg.robot_controller:main',
        'robot_estimator = warehouse_robot_controller_pkg.robot_estimator:main'
        ],
    },
)
