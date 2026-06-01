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
    maintainer='H.A. Sharif',
    maintainer_email='hiba2anwar@gmail.com',
    description='Hardware bridges for the CHROMA framework',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_bridge = chroma_bridges.telemetry_bridge:main',
            'degradation_manager = chroma_bridges.degradation_manager:main',
            'nav2_bridge = chroma_bridges.nav2_bridge:main'
        ],
    },
)
