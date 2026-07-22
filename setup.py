import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sentry_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'map'),
            glob('map/*.yaml') + glob('map/*.pgm')
            + glob('map/*.data') + glob('map/*.posegraph')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='baptisbc@rose-hulman.edu',
    description='Sentry localization (SLAM/AMCL/EKF) for RHIT Thornbots ARC 2026',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slam_relocalize_publisher = sentry_localization.slam_relocalize_publisher:main',
            'simple_relocalize_publisher = sentry_localization.simple_relocalize_publisher:main',
            'head_home_scan_gate = sentry_localization.head_home_scan_gate:main',
            'passthrough_odom_publisher = sentry_localization.passthrough_odom_publisher:main',
        ],
    },
)
