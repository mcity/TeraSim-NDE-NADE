from setuptools import setup, find_packages, Extension
from Cython.Build import cythonize
import numpy

extensions = [
    Extension(
        "terasim_nde_nade.vehicle.nde_vehicle_utils_cython",
        ["terasim_nde_nade/vehicle/nde_vehicle_utils_cython.pyx"],
        include_dirs=[numpy.get_include()],
    ),
    Extension(
        "*",
        ["terasim_nde_nade/**/*.py"],
    ),
]

setup(
    name="terasim_nde_nade",
    version="0.1.0",
    ext_modules=cythonize(extensions, build_dir="build"),
    package_data={
        "terasim_nde_nade": ["./build/*.so"],
    },
    zip_safe=False,
    install_requires=[
        "Cython>=3.0.8",
        "loguru>=0.5.3",
        "shapely>=2.0.3",
    ],
)
