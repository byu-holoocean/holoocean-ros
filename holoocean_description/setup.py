from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'holoocean_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (
            os.path.join('share', package_name, 'urdf/bluerov2_holoocean'),
            glob('urdf/bluerov2_holoocean/*.*'),
        ),
        (
            os.path.join('share', package_name, 'urdf/bluerov2_holoocean/meshes'),
            glob('urdf/bluerov2_holoocean/meshes/*.*'),
        ),
        (
            os.path.join('share', package_name, 'urdf/bluerov2_heavy'),
            glob('urdf/bluerov2_heavy/*.*'),
        ),
        (
            os.path.join('share', package_name, 'urdf/bluerov2_heavy/meshes'),
            glob('urdf/bluerov2_heavy/meshes/*.*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='URDF files for the CougUV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
