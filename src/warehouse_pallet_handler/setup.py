from setuptools import find_packages, setup

package_name = 'warehouse_pallet_handler'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='William Reishus',
    maintainer_email='williamreishus@gmail.com',
    description='Pallet attach/detach handler for the warehouse forklift.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pallet_handler_node = warehouse_pallet_handler.pallet_handler_node:main',
        ],
    },
)
