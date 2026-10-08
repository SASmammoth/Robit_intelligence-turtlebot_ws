from setuptools import find_packages, setup

package_name = 'test_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Terminal input publisher and subscriber printer for ROS 2 Jazzy',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pub_node = test_node.pub_node:main',
            'sub_node = test_node.sub_node:main',
        ],
    },
)
