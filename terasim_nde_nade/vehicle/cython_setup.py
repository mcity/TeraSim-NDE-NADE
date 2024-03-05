from setuptools import setup
from Cython.Build import cythonize
import numpy

setup(
    ext_modules = cythonize("nde_vehicle_utils_cython.pyx"),
    include_dirs=[numpy.get_include()]
)