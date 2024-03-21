python build.py # generate .so file
pyarmor gen -O dist_pyarmor -r -i ./terasim_nde_nade
python setup_pyarmor.py bdist_wheel