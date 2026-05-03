import os
from glob import glob

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    
    # Scripts and Launch
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'scripts'), glob('scripts/*.*')),
    
    # FORCE the worlds folder to copy .sdf files
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    
    # FORCE the pallet and its subfolders to copy
    (os.path.join('share', package_name, 'models/euro_pallet'), glob('models/euro_pallet/*.*')),
    (os.path.join('share', package_name, 'models/euro_pallet/meshes'), glob('models/euro_pallet/meshes/*.*')),
    (os.path.join('share', package_name, 'models/euro_pallet/materials/scripts'), glob('models/euro_pallet/materials/scripts/*.*')),
    (os.path.join('share', package_name, 'models/euro_pallet/materials/textures'), glob('models/euro_pallet/materials/textures/*.*')),
],