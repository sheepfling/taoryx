"""Setuptools hook that prevents stale namespace modules entering a wheel."""

from __future__ import annotations

from shutil import rmtree

from setuptools import setup
from setuptools.command.build_py import build_py as _BuildPy


class _CleanBuildPy(_BuildPy):
    """Rebuild this distribution's namespace tree from its declared sources."""

    def run(self) -> None:
        rmtree(self.build_lib, ignore_errors=True)
        super().run()
        ####

    ####


setup(cmdclass={"build_py": _CleanBuildPy})
