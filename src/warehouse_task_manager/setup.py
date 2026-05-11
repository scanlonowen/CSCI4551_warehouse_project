from setuptools import find_packages, setup

package_name = 'warehouse_task_manager'

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
    description='Stage 5.6 mission state machine for pallet pick-and-deliver.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission = warehouse_task_manager.mission:main',
        ],
    },
)
