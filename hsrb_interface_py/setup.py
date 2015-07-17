import sys
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hsrb_interface'],
    package_dir={'': 'src'}
)

install_requires = ['PyYAML']
if sys.version_info[0] < 3:
    install_requires.append('enum34')
d['install_requires'] = install_requires

setup(**d)
