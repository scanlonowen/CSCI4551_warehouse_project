from setuptools import find_packages, setup

package_name = 'warehouse_perception'

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
    description='RGB + LiDAR pallet detection for the warehouse forklift.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pallet_detector = warehouse_perception.pallet_detector:main',
        ],
    },
)
