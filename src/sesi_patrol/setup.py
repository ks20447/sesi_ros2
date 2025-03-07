import os 
from glob import glob
from setuptools import find_packages, setup

package_name = 'sesi_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "patrol_routes"), glob("patrol_routes/*"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam-morris',
    maintainer_email='adam@junction42.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "circle_patrol_exe = sesi_patrol.circle_patrol:main",
            "waypoint_patrol_exe = sesi_patrol.waypoint_patrol:main",
        ],
    },
)
