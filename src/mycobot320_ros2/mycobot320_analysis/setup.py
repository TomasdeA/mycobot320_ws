from setuptools import find_packages, setup

package_name = 'mycobot320_analysis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/show_configurations.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tomasdea',
    maintainer_email='tdeaguirre@fi.uba.ar',
    description='Tools to analyze the kinematics and singularities of the myCobot320 robot',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'show_configurations = mycobot320_analysis.show_configurations:main',
        ],
    },
)
