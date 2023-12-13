import os
from glob import glob
from setuptools import setup

package_name = 'uds_mmu_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyuhyong',
    maintainer_email='kyuhyong.you@nearthlab.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'visualizer = uds_mmu_teleop.visualizer:main',
                'teleop_commander = uds_mmu_teleop.teleop_commander:main',
                'teleop_keyboard = uds_mmu_teleop.teleop_keyboard:main',
                'teleop_joy = uds_mmu_teleop.teleop_joy:main'
        ],
    },
)
