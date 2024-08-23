from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'holoocean_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.[pxy][yma]*'))),   
        (os.path.join('share',package_name,'config'),glob('config/*.json')),  
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),  
        (os.path.join('share',package_name,'scripts'),glob('scripts/*.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bradenmeyers',
    maintainer_email='bjm255@byu.edu',
    description='Ros Interface for Holoocean Marine Simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'holoocean_node = holoocean_main.holoocean_node:main',
            'torpedo_node = holoocean_main.torpedo_node:main',
            'state_estimate = holoocean_main.RK45_state_est:main',
            'command_node = holoocean_main.command_node:main',
            'fins_node = holoocean_main.fins_node:main',
            'controller_node = holoocean_main.controller_node:main'
        ],
    },
)
