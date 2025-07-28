from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'mycobot320_analysis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'sympy',
        'matplotlib',
        'spatialmath-python',
        'roboticstoolbox-python'
    ],

    zip_safe=True,
    maintainer='tomasdea',
    maintainer_email='tdeaguirre@fi.uba.ar',
    description='Tools to analyze the kinematics and singularities of the myCobot320 robot',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'show_configurations = mycobot320_analysis.nodes.show_configurations:main',
        ],
    },
)
