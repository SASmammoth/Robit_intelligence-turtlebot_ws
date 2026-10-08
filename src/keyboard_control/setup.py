from setuptools import find_packages, setup

package_name = 'keyboard_control'

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
    description='w/a/s/d keyboard velocity publisher with start handshake',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_node = keyboard_control.keyboard_node:main',
        ],
    },
)
