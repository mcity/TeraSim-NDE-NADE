from setuptools import setup, find_packages

setup(
    name='terasim_nde_ite',
    version='0.1',
    author='Haowei Sun',
    author_email='haoweis@umich.edu',
    packages=["terasim_nde_ite", "terasim_nde_ite.envs", "terasim_nde_ite.vehicle"],
    description='TeraSim NDE ITE package',
    url='https://github.com/michigan-traffic-lab/TeraSim-NDE-ITE',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
)