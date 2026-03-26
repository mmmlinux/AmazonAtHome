import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'logistics_web'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Web UI bridge for the warehouse logistics system',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_node = logistics_web.web_node:main',
        ],
    },
)
