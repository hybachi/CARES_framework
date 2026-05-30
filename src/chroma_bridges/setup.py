from setuptools import find_packages, setup

package_name = 'chroma_bridges'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'telemetry_bridge = chroma_bridges.telemetry_bridge:main',
            'cmd_vel_bridge = chroma_bridges.cmd_vel_bridge:main',
            'nav2_bridge = chroma_bridges.nav2_bridge:main'
        ],
    },
)
