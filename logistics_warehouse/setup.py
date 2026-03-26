from setuptools import find_packages, setup

package_name = 'logistics_warehouse'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Warehouse slot and dock occupancy state manager',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'warehouse_state = logistics_warehouse.warehouse_state_node:main',
        ],
    },
)
