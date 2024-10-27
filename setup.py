from Cython.Build import cythonize
from setuptools import find_packages, setup

setup(
    name="kbm",
    packages=find_packages(include=["kbm", "kbm.*"]),
    ext_modules=cythonize("kbm/*.pyx"),  # pyright: ignore [reportUnknownArgumentType]
)
