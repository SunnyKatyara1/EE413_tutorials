from setuptools import setup
from Cython.Build import cythonize

setup(
    name='parametric_policy',
    ext_modules=cythonize("parametric_policy.py"),
)

