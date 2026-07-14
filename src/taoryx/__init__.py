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
    table_explorer,
    tables,
    visualization,
)
from .modes import DynamicsMode, Kinematic6DofState, Quaternion
from .outputs import DynamicsKind, EventRecord, RunArtifact, SegmentSpan, TelemetryChannel, VehicleKind, VehicleTelemetry, build_run_artifact
from .state import PointMassRates, PointMassState
from .table_explorer import (
    AxisInterpolationBracket,
    InterpolationExplanation,
    TableInspection,
    TableInspectionArtifact,
    TableInspectionFormat,
    TableInspectionStatus,
    explain_interpolation,
    inspect_table_document,
    inspect_table_file,
)

__all__ = ["__version__", "aerodynamics", "atmosphere", "attitude", "contracts", "coordinates", "earth", "equations", "forces", "geodesy", "gravity", "guidance", "iip", "language", "linalg", "modes", "numeric", "optimization", "outputs", "radar", "runtime", "searches", "simulation", "state", "state_rates", "table_explorer", "tables", "visualization", "AxisInterpolationBracket", "DynamicsKind", "DynamicsMode", "EventRecord", "InterpolationExplanation", "Kinematic6DofState", "PointMassRates", "PointMassState", "Quaternion", "RunArtifact", "SegmentSpan", "TableInspection", "TableInspectionArtifact", "TableInspectionFormat", "TableInspectionStatus", "TelemetryChannel", "VehicleKind", "VehicleTelemetry", "build_run_artifact", "explain_interpolation", "inspect_table_document", "inspect_table_file"]
