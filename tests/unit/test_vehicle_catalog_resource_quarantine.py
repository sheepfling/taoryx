"""Regression checks for selected-catalog versus compatibility resource loading."""

from __future__ import annotations

import subprocess
import sys


def test_selected_catalog_import_does_not_eagerly_load_compatibility_resolver() -> None:
    """Core contract imports must not consult the aggregate package ordering."""

    script = """
import sys

import taoryx.vehicle_composition_registry

assert "taoryx.compatibility.vehicle_catalog_resources" not in sys.modules
from taoryx.vehicle_composition_registry import VEHICLE_COMPOSITION_REGISTRY
assert VEHICLE_COMPOSITION_REGISTRY.name == "vehicle_composition_registry.yaml"
assert "taoryx.compatibility.vehicle_catalog_resources" in sys.modules
"""
    completed = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    ####
