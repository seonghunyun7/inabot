from setuptools import find_packages, setup

package_name = 'battery_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can', 'aenum'],
    zip_safe=True,
    maintainer='ysh',
    maintainer_email='seonghunyun7@gmail.com',
    description='Battery CAN ROS2 Python node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_can_node = battery_can.battery_can_node:main',
        ],
    },
)
