"""Python successor to the TAOS 1995 simulation system."""

__version__ = "0.1.0.dev0"

from . import (
    aerodynamics,
    atmosphere,
    attitude,
    contracts,
    coordinates,
    earth,
    equations,
    forces,
    geodesy,
    gravity,
    guidance,
    iip,
    language,
    linalg,
    modes,
    numeric,
    optimization,
    outputs,
    radar,
    runtime,
    searches,
    simulation,
    state,
    state_rates,
    tables,
    visualization,
)
from .modes import DynamicsMode, Kinematic6DofState, Quaternion
from .outputs import DynamicsKind, EventRecord, RunArtifact, SegmentSpan, TelemetryChannel, VehicleKind, VehicleTelemetry, build_run_artifact
from .state import PointMassRates, PointMassState

__all__ = ["__version__", "aerodynamics", "atmosphere", "attitude", "contracts", "coordinates", "earth", "equations", "forces", "geodesy", "gravity", "guidance", "iip", "language", "linalg", "modes", "numeric", "optimization", "outputs", "radar", "runtime", "searches", "simulation", "state", "state_rates", "tables", "visualization", "DynamicsKind", "DynamicsMode", "EventRecord", "Kinematic6DofState", "PointMassRates", "PointMassState", "Quaternion", "RunArtifact", "SegmentSpan", "TelemetryChannel", "VehicleKind", "VehicleTelemetry", "build_run_artifact"]
