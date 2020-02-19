#!/usr/bin/env python3

"""The setup script."""

import pathlib
from setuptools import setup, find_packages

here = pathlib.Path(__file__).parent

with open(here / "yaqd_adafruit" / "VERSION") as version_file:
    version = version_file.read().strip()


with open("README.md") as readme_file:
    readme = readme_file.read()


requirements = ["yaqd-core", "adafruit-circuitpython-motorkit"]

extra_requirements = {"dev": ["black", "pre-commit"]}

setup_requirements = [
    "pytest-runner",
]

test_requirements = [
    "pytest>=3",
]
extra_files = {"yaqd_adafruit": ["VERSION"]}

setup(
    author="yaq Developers",
    author_email="git@ksunden.space",
    python_requires=">=3.7",
    classifiers=[
        "Development Status :: 2 - Pre-Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)",
        "Natural Language :: English",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Topic :: Scientific/Engineering",
    ],
    description="Daemons for Adafruit products",
    entry_points={
        "console_scripts": [
            "yaqd-stepper-motor-hat=yaqd_adafruit._stepper_motor_hat:StepperMotorHat.main",
        ],
    },
    install_requires=requirements,
    extras_require=extra_requirements,
    license="GNU Lesser General Public License v3 (LGPL)",
    long_description=readme,
    long_description_content_type="text/markdown",
    include_package_data=True,
    package_data=extra_files,
    keywords="yaqd-adafruit",
    name="yaqd-adafruit",
    packages=find_packages(include=["yaqd-adafruit", "yaqd-adafruit.*"]),
    setup_requires=setup_requirements,
    test_suite="tests",
    tests_require=test_requirements,
    url="https://gitlab.com/yaq/yaqd-adafruit",
    version=version,
    zip_safe=False,
)
