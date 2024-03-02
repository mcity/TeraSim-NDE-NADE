from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules = cythonize("nde_vehicle_utils_cython.pyx")
)