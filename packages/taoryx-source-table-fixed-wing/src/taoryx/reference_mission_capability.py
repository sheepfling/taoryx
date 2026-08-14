"""Compatibility import path for source-table fixed-wing mission capability.

The historical module name remains importable from the X8/B747 owner wheel.
New package-local code should use :mod:`taoryx.source_table_fixed_wing_mission_capability`.
"""

from __future__ import annotations

from .source_table_fixed_wing_mission_capability import (
    B747SourceRacetrackCapabilityAdapter,
    X8SourceRacetrackCapabilityAdapter,
    compile_b747_source_direct_wrench_racetrack_from_composition,
    compile_x8_source_direct_wrench_racetrack_from_composition,
    reference_mission_capability_adapters,
)

__all__ = [
    "B747SourceRacetrackCapabilityAdapter",
    "X8SourceRacetrackCapabilityAdapter",
    "compile_b747_source_direct_wrench_racetrack_from_composition",
    "compile_x8_source_direct_wrench_racetrack_from_composition",
    "reference_mission_capability_adapters",
]
