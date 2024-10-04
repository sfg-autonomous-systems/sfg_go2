import os
from glob import glob

from setuptools import find_packages, setup

package_name = "go2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Constantin Blessing",
    maintainer_email="constantin.blessing@hs-esslingen.de",
    description="The Go2 package.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera = go2.camera:main",
        ],
    },
)
