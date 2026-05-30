import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'chroma_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hybachi',
    maintainer_email='hiba2anwar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'capability_manager = chroma_core.capability_manager:main',
            'task_allocator = chroma_core.task_allocator:main',
            'mission_spawner = chroma_core.mission_spawner:main',
        ],
    },
)
