from setuptools import setup, find_packages

setup(
    name="terasim_nde_nade",
    version="0.1.0",
    packages=find_packages(where="dist_pyarmor"),
    package_dir={"": "dist_pyarmor"},
    package_data={
        "terasim_nde_nade": ["**/*.so"],
    },
    zip_safe=False,
)
