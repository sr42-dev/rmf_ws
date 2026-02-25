from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rmf_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='Integration tests for the RMF stack',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_navigation = rmf_tests.test_navigation:main',
            'test_patrol_task = rmf_tests.test_patrol_task:main',
        ],
    },
)
