from setuptools import find_packages, setup

package_name = 'logistics_robot_sim'

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
    description='Simulated robot for the warehouse logistics system',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_sim = logistics_robot_sim.robot_sim:main',
        ],
    },
)
