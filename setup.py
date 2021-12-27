# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["system_stats"], package_dir={"": "src"}
)

setup(**setup_args)


from setuptools import setup

package_name = 'system_stats'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Saba',
    maintainer_email='asaba96@pitt.edu',
    description='ROS node around PSUtil that fetches system stats & publishes to ROS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system-monitor = py_pubsub.publisher_member_function:main',
        ],
    },
)
