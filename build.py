import numpy
from Cython.Build import cythonize
from setuptools import find_packages, setup
from setuptools.extension import Extension

extensions = [
    Extension(
        "terasim_nde_nade.utils.collision.collision_check_cy",
        ["terasim_nde_nade/utils/collision/collision_check_cy.pyx"],
        include_dirs=[numpy.get_include()],
    ),
    Extension(
        "terasim_nde_nade.utils.geometry.geometry_utils_cy",
        ["terasim_nde_nade/utils/geometry/geometry_utils_cy.pyx"],
        include_dirs=[numpy.get_include()],
    ),
    Extension(
        "terasim_nde_nade.utils.trajectory.trajectory_utils_cy",
        ["terasim_nde_nade/utils/trajectory/trajectory_utils_cy.pyx"],
        include_dirs=[numpy.get_include()],
    ),
]

setup(ext_modules=cythonize(extensions), script_args=["build_ext", "--inplace"])
