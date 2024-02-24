from setuptools import setup, find_packages

setup(
    name='terasim_nde_nade',
    version='0.1',
    author='Haowei Sun',
    author_email='haoweis@umich.edu',
    packages=["terasim_nde_nade", "terasim_nde_nade.envs", "terasim_nde_nade.vehicle"],
    package_data={
        'terasim_nde_nade.vehicle': ['lane_config.json'],
    },
    description='TeraSim NDE ITE package',
    url='https://github.com/michigan-traffic-lab/TeraSim-NDE-ITE',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
)
