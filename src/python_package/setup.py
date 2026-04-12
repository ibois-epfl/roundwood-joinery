
from setuptools import setup, find_packages
import os
__version__ = open(os.path.join(os.path.dirname(__file__), "../../version")).read().strip()

setup(
    name="roundwood_joinery",
    version=__version__,
    packages=find_packages(),
    install_requires=[
        "numpy==2.0.2",
        # other dependencies...
    ],
    description="roundwood_joinery is a python package to optimize placement of joints in a roundwood elements.",
    long_description=open("../../README.md").read(),
    long_description_content_type="text/markdown",
    author="Damien Gilliard",
    author_email="damien.gilliard@epfl.ch",
    url="https://github.com/ibois-epfl/roundwood-joinery",
    classifiers=[
        "License :: OSI Approved :: GPL-3.0 License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
    ],
    include_package_data=True,
    package_data={  # type: ignore[misc]
        "roundwood_joinery": ["roundwood_joinery/*.dll", "roundwood_joinery/*.pyd"]
    },
)
