from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rostrace'],
    package_dir={'': 'src'},
    package_data={'rostrace': ['bpf/*.bt']},
    entry_points={'console_scripts': ['rostrace=rostrace.main:main']})

setup(**d)
