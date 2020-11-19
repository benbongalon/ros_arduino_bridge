import os
from glob import glob
from setuptools import setup

package_name = 'ros_arduino_python'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        # Include all Nodes in library folder
        (os.path.join('lib', package_name), glob('nodes/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ben Bongalon',
    author_email='ben.bongalon@gmail.com',
    maintainer='Ben Bongalon',
    maintainer_email='ben.bongalon@gmail.com',
    keywords=['ROS 2', 'Arduino'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Python3 interface for the ROS Arduino Bridge',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = ros_arduino_python.demo:main',
            'driver = ros_arduino_python.arduino_driver:main'
        ],
    },
)
