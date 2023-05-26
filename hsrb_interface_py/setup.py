# Copyright (C) 2022 Toyota Motor Corporation
import os

from setuptools import setup

package_name = 'hsrb_interface_py'

setup(
    name=package_name,
    version='0.12.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keisuke Takeshita',
    maintainer_email='keisuke_takeshita@mail.toyota.co.jp',
    description='Python interfaces scripts',
    license='TMC',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ihsrb = hsrb_interface_py.ihsrb:main',
        ],
    },
)
