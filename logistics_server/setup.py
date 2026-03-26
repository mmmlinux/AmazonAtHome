import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'logistics_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Navigation server for the warehouse logistics system',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nav_server = logistics_server.nav_server:main',
            'task_client = logistics_server.task_client:main',
        ],
    },
)
