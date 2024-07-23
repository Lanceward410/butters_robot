import os
from setuptools import setup
from glob import glob

package_name = 'tcp_communications_pkg'

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
    description='TCP Communications package for UMES robots',
    license='Contact me and let me know what you want to use this for! Dont just steal it, maybe I can help you learn/understand/implement/improve it.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'client_node = tcp_communications_pkg.client_node:main',
         'server_node = tcp_communications_pkg.server_node:main'
        ],
    },
)
