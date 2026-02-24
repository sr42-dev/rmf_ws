from setuptools import setup
from glob import glob
import os

package_name = 'traffic_editor_assets'

# Resolve through symlinks to get the real source directory.
# With colcon --symlink-install, setup.py is symlinked from the build dir
# back to the source dir, so realpath gives us the source location.
src_dir = os.path.dirname(os.path.realpath(__file__))

# Create a list of all files in the assets directory
data_files = [x for x in glob(os.path.join(src_dir, 'assets', '**'), recursive=True)
              if os.path.isfile(x)]

# Compute data_files entries preserving subdirectory structure under share/
data_file_appends = [
    ('share/' + package_name + '/' + os.path.relpath(os.path.dirname(x), src_dir), [x])
    for x in data_files
]

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join(src_dir, 'resource', package_name)]),
        ('share/ament_index/resource_index/traffic_editor_assets',
            [os.path.join(src_dir, 'resource', 'assets')]),
        ('share/' + package_name, [os.path.join(src_dir, 'package.xml')]),
        ('share/' + package_name, [
            os.path.join(src_dir, 'map.building.yaml'),
            os.path.join(src_dir, 'map.png'),
        ]),
        *data_file_appends
    ],
    install_requires=['setuptools'],
    # zip_safe=True,
    author='Brandon Ong',
    author_email='brandon@osrfoundation.org',
    maintainer='Brandon Ong',
    maintainer_email='brandon@osrfoundation.org',
    keywords=['RMF', 'traffic_editor'],
    classifiers=[
        'Intended Audience :: End-Users',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: None',
        'Topic :: Simulator World Development',
    ],
    description='Assets for use with traffic_editor.',
    license='Apache License, Version 2.0',
    tests_require=[],
    entry_points={},
)
