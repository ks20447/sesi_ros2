from setuptools import find_packages, setup

package_name = 'sesi_emitter'

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
    maintainer='adam-morris',
    maintainer_email='adam@junction42.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "signal_emitter_server = sesi_emitter.signal_emitter_server:main",
            "signal_emitter_client = sesi_emitter.signal_emitter_client:main",
            "grid_publisher_server = sesi_emitter.signal_strength_grid_publisher_server:main",
        ],
    },
)
