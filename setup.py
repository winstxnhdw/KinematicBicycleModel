from Cython.Build import cythonize
from setuptools import Extension, find_packages, setup

extensions = [
    Extension("kbm.model", ["kbm/model/model.pyx"]),
]

setup(
    name="kbm",
    packages=find_packages(include=["kbm", "kbm.*"]),
    ext_modules=cythonize(extensions),  # pyright: ignore [reportUnknownArgumentType]
)
