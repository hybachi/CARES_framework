import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'chroma_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'config', 'nav2'),
            glob('config/nav2/*.yaml')),
        (os.path.join('share', package_name, 'config', 'maps'),
            glob('config/maps/*')),
        (os.path.join('share', package_name, 'config', 'profiles'),
            glob('config/profiles/*.yaml')),

        # Install all launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Install mapping directory
        (os.path.join('share', package_name, 'mapping'),
            glob('mapping/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hybachi',
    maintainer_email='hiba2anwar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'scan_resampler = chroma_bringup.scan_resampler:main',
        ],
    },
)
