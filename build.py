from setuptools import setup, find_packages
from setuptools.extension import Extension
from setuptools.command.build_ext import build_ext
from Cython.Build import cythonize
import numpy

extensions = [
    Extension(
        "terasim_nde_nade.vehicle.nde_vehicle_utils_cython",
        ["terasim_nde_nade/vehicle/nde_vehicle_utils_cython.pyx"],
        include_dirs=[numpy.get_include()],
    ),
]


def build(setup_kwargs):
    setup_kwargs.update(
        ext_modules=cythonize(extensions),
        cmdclass={"build_ext": build_ext},
    )
