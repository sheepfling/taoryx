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
    scenario,
    searches,
    simulation,
    state,
    state_rates,
    table_explorer,
    tables,
    validation,
    visualization,
)
from .family_debug_rendering import FamilyDebugRenderReport, render_family_debug_artifacts
from .family_debugging import DebugFamily, FamilyDebugPlan, build_family_debug_plan, family_profile
from .modes import DynamicsMode, Kinematic6DofState, Quaternion
from .outputs import DynamicsKind, EventRecord, RunArtifact, SegmentSpan, TelemetryChannel, VehicleKind, VehicleTelemetry, build_run_artifact
from .rigid_body import (
    RIGID_BODY_STATE_NAMES,
    RigidBody6DofModel,
    RigidBody6DofState,
    RigidBodyForceMoment,
    ThermalAssessment,
    ThermalLimits,
    assess_thermal_limits,
)
from .rigid_body_frames import EarthRotationAdapter
from .rotorcraft import QuadRotorAllocation, RotorCommandSet
from .scenario import (
    ControlContract,
    OutputContract,
    RandomSeed,
    ResolutionRecord,
    ResolvedScenario,
    ScenarioCompileError,
    ScenarioCompiler,
    ScenarioRequest,
    ScenarioRuntimeContract,
    ScenarioSource,
    StatusContract,
)
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
from .validation import Direction, PhaseWindow, require_bounded, require_change_of_sign, require_channel, require_monotonic, require_net_change
from .vehicle import (
    AerodynamicOutput,
    AeroQueryContext,
    MassProperties,
    PreparedAerodynamicCoefficients,
    PreparedCoefficientTable,
    PropulsionOutput,
    StageDefinition,
    StagedPropulsion,
    TableAerodynamicModel,
)
from .visualization import render_run_artifact_html, render_run_artifact_plots

__all__ = ["__version__", "aerodynamics", "atmosphere", "attitude", "contracts", "coordinates", "earth", "equations", "forces", "geodesy", "gravity", "guidance", "iip", "language", "linalg", "modes", "numeric", "optimization", "outputs", "radar", "rigid_body", "rigid_body_frames", "rotorcraft", "runtime", "scenario", "searches", "simulation", "state", "state_rates", "table_explorer", "tables", "validation", "vehicle", "visualization", "AeroQueryContext", "AerodynamicOutput", "AxisInterpolationBracket", "ControlContract", "DebugFamily", "Direction", "DynamicsKind", "DynamicsMode", "EarthRotationAdapter", "EventRecord", "FamilyDebugPlan", "FamilyDebugRenderReport", "InterpolationExplanation", "Kinematic6DofState", "MassProperties", "OutputContract", "PhaseWindow", "PointMassRates", "PointMassState", "PreparedAerodynamicCoefficients", "PreparedCoefficientTable", "PropulsionOutput", "QuadRotorAllocation", "Quaternion", "RIGID_BODY_STATE_NAMES", "RandomSeed", "ResolutionRecord", "ResolvedScenario", "RigidBody6DofModel", "RigidBody6DofState", "RigidBodyForceMoment", "RotorCommandSet", "RunArtifact", "ScenarioCompileError", "ScenarioCompiler", "ScenarioRequest", "ScenarioRuntimeContract", "ScenarioSource", "SegmentSpan", "StageDefinition", "StagedPropulsion", "StatusContract", "TableAerodynamicModel", "TableInspection", "TableInspectionArtifact", "TableInspectionFormat", "TableInspectionStatus", "TelemetryChannel", "ThermalAssessment", "ThermalLimits", "VehicleKind", "VehicleTelemetry", "assess_thermal_limits", "build_family_debug_plan", "build_run_artifact", "explain_interpolation", "family_profile", "inspect_table_document", "inspect_table_file", "render_family_debug_artifacts", "render_run_artifact_html", "render_run_artifact_plots", "require_bounded", "require_change_of_sign", "require_channel", "require_monotonic", "require_net_change"]
