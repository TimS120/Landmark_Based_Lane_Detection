from setuptools import find_packages, setup
import os, glob

package_name = 'landmark_based_lane_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tim1362s@hs-coburg.de',
    maintainer_email='tim.strobel@stud.hs-coburg.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pylon_detection_node = landmark_based_lane_detection_pkg.pylon_detection:main'
        ],
    },
)
