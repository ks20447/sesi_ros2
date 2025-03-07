import os
from glob import glob
from setuptools import find_packages, setup

package_name = "sesi_nav"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config_files"), glob("config_files/*")),
        (
            os.path.join("share", package_name, "config_templates"),
            glob("config_templates/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="adam-morris",
    maintainer_email="adam@junction42.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "generate_config_files_exe = sesi_nav.config_generate:main",
        ],
    },
)
