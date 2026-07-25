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
    reachability_catalog,
    reachability_envelope,
    reachability_visualization,
    runtime,
    scenario,
    searches,
    segmentation,
    simulation,
    state,
    state_rates,
    table_explorer,
    tables,
    validation,
    visualization,
    x15_reachability,
)
from .control_directions import ControlDirectionProbe, ControlDirectionResult, audit_control_directions
from .controller_autotune import (
    AutoTuneCandidate,
    AutoTuneLimits,
    AutoTuneReport,
    auto_tune_lqr_profiles,
    default_attitude_linearization,
)
from .controller_design import (
    ControllerDesignCatalog,
    ControllerDesignMethod,
    ControllerDesignSpec,
    build_lqr_controller,
    load_controller_catalog,
)
from .family_debug_rendering import FamilyDebugRenderReport, render_family_debug_artifacts
from .family_debugging import DebugFamily, FamilyDebugPlan, build_family_debug_plan, family_profile
from .modes import DynamicsMode, Kinematic6DofState, Quaternion
from .objectives import ObjectiveResult, ObjectiveSpec, score_objective, score_objectives
from .outputs import DynamicsKind, EventRecord, RunArtifact, SegmentSpan, TelemetryChannel, VehicleKind, VehicleTelemetry, build_run_artifact
from .reachability_catalog import (
    ReachabilityCatalog,
    ReachabilityCommonObject,
    ReachabilityFamilySpec,
    ReachabilityProfileSpec,
    ReachabilityStudySemantic,
    load_reachability_catalog,
)
from .reachability_envelope import (
    EnvelopeBounds,
    EnvelopeSample,
    EnvelopeTermination,
    LaunchCommand,
    PointMass3DofState,
    Pseudo6DofState,
    ReachabilityEnvelope,
    ReachabilityFidelity,
    ReachabilitySearchSpace,
    RocketGlideVehicle,
    SearchAxis,
    TerminalCriteria,
    TrajectoryResult,
    generate_launch_grid,
    run_reachability_envelope,
    simulate_rocket_glide,
)
from .reachability_visualization import (
    ReachabilityPlotReport,
    load_reachability_artifact,
    plot_fidelity_progression,
    plot_flown_trajectories,
    plot_search_coverage,
    plot_terminal_capability,
    render_reachability_plot_bundle,
)
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
from .segmentation import (
    GoalSpec,
    SegmentationCatalog,
    SegmentationScenario,
    SegmentSpec,
    TransitionEventSpec,
    TransitionPolicy,
    transition_audit,
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
from .trim import DynamicsLinearization, TrimResult, TrimSpec, finite_difference_dynamics_linearization, finite_difference_linearization, solve_trim
from .trim_catalog import TrimCatalog, TrimCatalogEntry, load_trim_catalog
from .validation import (
    Direction,
    PhaseWindow,
    actuator_saturation_fraction,
    capture_time,
    dwell_in_band,
    energy_balance_residual,
    independent_force_closure,
    integral_mass_balance_error,
    phase_slice,
    require_bounded,
    require_change_of_sign,
    require_channel,
    require_monotonic,
    require_net_change,
    settling_time,
    specific_energy,
    timestep_convergence_error,
    wrapped_angle_error,
)
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
from .vehicle_onboarding import (
    OnboardingFinding,
    VehicleOnboardingReport,
    validate_all_vehicle_onboarding,
    validate_vehicle_onboarding,
)
from .visualization import render_run_artifact_html, render_run_artifact_plots
from .x15_reachability import (
    X15ReachabilityBundle,
    run_x15_reachability_tiers,
    write_x15_reachability_bundle,
    x15_reachability_commands,
    x15_surrogate_vehicle,
)

__all__ = ["__version__", "AutoTuneCandidate", "AutoTuneLimits", "AutoTuneReport", "ReachabilityCatalog", "ReachabilityCommonObject", "ReachabilityFamilySpec", "ReachabilityProfileSpec", "ReachabilityStudySemantic", "EnvelopeBounds", "EnvelopeSample", "EnvelopeTermination", "LaunchCommand", "PointMass3DofState", "Pseudo6DofState", "ReachabilityEnvelope", "ReachabilityFidelity", "ReachabilitySearchSpace", "RocketGlideVehicle", "SearchAxis", "TerminalCriteria", "TrajectoryResult", "ReachabilityPlotReport", "load_reachability_artifact", "plot_fidelity_progression", "plot_flown_trajectories", "plot_search_coverage", "plot_terminal_capability", "render_reachability_plot_bundle", "X15ReachabilityBundle", "run_x15_reachability_tiers", "write_x15_reachability_bundle", "x15_reachability_commands", "x15_surrogate_vehicle", "generate_launch_grid", "run_reachability_envelope", "simulate_rocket_glide", "auto_tune_lqr_profiles", "default_attitude_linearization", "OnboardingFinding", "VehicleOnboardingReport", "load_reachability_catalog", "validate_all_vehicle_onboarding", "validate_vehicle_onboarding", "aerodynamics", "atmosphere", "attitude", "contracts", "coordinates", "earth", "equations", "forces", "geodesy", "gravity", "guidance", "iip", "language", "linalg", "modes", "numeric", "objectives", "optimization", "outputs", "radar", "reachability_catalog", "reachability_envelope", "reachability_visualization", "x15_reachability", "rigid_body", "rigid_body_frames", "rotorcraft", "runtime", "scenario", "segmentation", "searches", "simulation", "state", "state_rates", "table_explorer", "tables", "trim", "validation", "vehicle", "visualization", "AeroQueryContext", "AerodynamicOutput", "AxisInterpolationBracket", "ControlContract", "ControlDirectionProbe", "ControlDirectionResult", "ControllerDesignCatalog", "ControllerDesignMethod", "ControllerDesignSpec", "DebugFamily", "Direction", "DynamicsKind", "DynamicsLinearization", "DynamicsMode", "EarthRotationAdapter", "EventRecord", "FamilyDebugPlan", "GoalSpec", "FamilyDebugRenderReport", "InterpolationExplanation", "Kinematic6DofState", "MassProperties", "ObjectiveResult", "ObjectiveSpec", "OutputContract", "PhaseWindow", "PointMassRates", "PointMassState", "PreparedAerodynamicCoefficients", "PreparedCoefficientTable", "PropulsionOutput", "QuadRotorAllocation", "Quaternion", "RIGID_BODY_STATE_NAMES", "RandomSeed", "ResolutionRecord", "ResolvedScenario", "RigidBody6DofModel", "RigidBody6DofState", "RigidBodyForceMoment", "RotorCommandSet", "RunArtifact", "ScenarioCompileError", "ScenarioCompiler", "ScenarioRequest", "ScenarioRuntimeContract", "ScenarioSource", "SegmentSpan", "SegmentSpec", "SegmentationCatalog", "SegmentationScenario", "StageDefinition", "StagedPropulsion", "StatusContract", "TableAerodynamicModel", "TableInspection", "TableInspectionArtifact", "TableInspectionFormat", "TableInspectionStatus", "TelemetryChannel", "ThermalAssessment", "ThermalLimits", "TransitionEventSpec", "TransitionPolicy", "TrimCatalog", "TrimCatalogEntry", "TrimResult", "TrimSpec", "VehicleKind", "VehicleTelemetry", "actuator_saturation_fraction", "assess_thermal_limits", "audit_control_directions", "build_family_debug_plan", "build_lqr_controller", "build_run_artifact", "capture_time", "dwell_in_band", "energy_balance_residual", "explain_interpolation", "family_profile", "finite_difference_dynamics_linearization", "finite_difference_linearization", "independent_force_closure", "inspect_table_document", "inspect_table_file", "integral_mass_balance_error", "load_controller_catalog", "load_trim_catalog", "phase_slice", "render_family_debug_artifacts", "render_run_artifact_html", "render_run_artifact_plots", "require_bounded", "require_change_of_sign", "require_channel", "require_monotonic", "require_net_change", "score_objective", "score_objectives", "require_net_change", "settling_time", "solve_trim", "specific_energy", "timestep_convergence_error", "transition_audit", "wrapped_angle_error"]
