from setuptools import find_packages, setup

package_name = "example"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Constantin Blessing",
    maintainer_email="constantin.blessing@hs-esslingen.de",
    description="An example package.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "butt_wiggler = example.butt_wiggler:main",
            "camera_recorder = example.camera_recorder:main",
        ],
    },
)
