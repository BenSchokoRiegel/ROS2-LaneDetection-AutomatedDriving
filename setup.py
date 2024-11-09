from setuptools import find_packages, setup
package_name = 'lane_detection'

from generate_parameter_library_py.setup_helper import generate_parameter_module
generate_parameter_module(
    "image_params",
    "config/params.yaml"
)

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
    maintainer='CiticarGroup',
    maintainer_email='koroyeldiores@gmail.com',
    description='TODO: Package description',
    license='Apache 2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_lanelet = lane_detection.image_lanelet:main'
        ],
    },
)
