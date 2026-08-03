"""Python successor to the TAOS 1995 simulation system."""

__version__ = "0.1.0.dev0"

from . import (
    aerodynamics,
    atmosphere,
    attitude,
    composition_episode,
    composition_policy,
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
    showcase,
    simulation,
    state,
    state_rates,
    table_explorer,
    tables,
    validation,
    visualization,
    x15_reachability,
)
from .composition_episode import (
    EpisodeChannel,
    EpisodeObservation,
    EpisodeStep,
    HummingbirdPseudoCompositionEpisode,
    LanguageBackedCompositionEpisode,
    VehicleCompositionEpisode,
    open_vehicle_composition_episode,
)
from .composition_policy import CompositionPolicyTrace, PolicyDecision, PolicyFunction, run_composition_policy
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
from .controller_inventory import ControllerInventory, ControllerInventoryEntry
from .controller_realization import (
    AllocationResult,
    ClosedLoopPole,
    ControllerChannel,
    ControllerPreflightIssue,
    ControllerPreflightReport,
    ControllerProvenance,
    ControllerQualificationStatus,
    ControllerRealization,
    ControllerRuntimeState,
    ControllerSchedule,
    GeneralizedControlRequest,
    GuidanceReference,
    preflight_controller_realization,
)
from .controller_registry import (
    ControllerBackendFactory,
    ControllerBackendRegistry,
    ControllerBackendSpec,
    ControllerBackendStatus,
    default_controller_backend_registry,
)
from .direct_wrench import (
    DIRECT_WRENCH_NAMES,
    DirectWrenchLimits,
    DirectWrenchProjection,
    add_direct_wrench_to_local_derivative,
    compose_direct_wrench_load,
)
from .family_adapter import (
    AdapterCapability,
    AdapterCapabilityError,
    AdapterChannel,
    AdapterConformanceFinding,
    AdapterConformanceReport,
    AllocationProvider,
    EffectivenessProvider,
    FamilyAdapter,
    FamilyAdapterDescriptor,
    FamilyCapabilityReport,
    StandardFamilyAdapter,
    StateDerivativeProvider,
    descriptor_from_control_plant,
    descriptor_from_direct_wrench_state,
    validate_family_adapter,
)
from .family_adapter_probes import AdapterProbeCase, AdapterProbeOperation, AdapterProbeReport, run_adapter_probe
from .family_adapter_registry import (
    AdapterRegistrationCheck,
    AdapterRegistrationError,
    AdapterRegistryReport,
    FamilyAdapterProbeFactory,
    FamilyAdapterRegistration,
    FamilyAdapterRegistry,
)
from .family_debug_rendering import FamilyDebugRenderReport, render_family_debug_artifacts
from .family_debugging import DebugFamily, FamilyDebugPlan, build_family_debug_plan, family_profile
from .family_manifest import (
    UnifiedFamilyManifest,
    UnifiedFamilyManifestCatalog,
    UnifiedFamilyManifestFinding,
    load_unified_family_manifest_catalog,
)
from .family_strategy import (
    FAMILY_STRATEGY_CATALOG,
    FamilyIntegrationStrategy,
    FamilyIntegrationStrategyCatalog,
    FamilyStrategyConformanceReport,
    FamilyStrategyWorkItem,
    FamilyStrategyWorklistReport,
    FamilyTierStrategy,
    build_family_strategy_worklist,
    load_family_strategy_catalog,
    validate_family_strategy_catalog,
)
from .fidelity_contracts import (
    CANONICAL_FIDELITY_TIERS,
    FIDELITY_TIER_RANK,
    LEGACY_FIDELITY_ORDER,
    canonical_tier_for_runtime,
    canonicalize_fidelity,
    control_realization_for,
    parent_fidelity,
    runtime_fidelity_for,
)
from .fidelity_lowering import LoweringCandidate, LoweringDecision, select_canonical_lowering
from .generic_tuning import (
    AuthorityPreflightReport,
    GenericLqrCandidate,
    GenericLqrProfile,
    GenericLqrReport,
    LinearAuthorityRequirement,
    TrimToTuneResult,
    linear_authority_preflight,
    linear_authority_preflight_evaluator,
    trim_linearize_and_tune,
    tune_lqr_profiles,
)
from .hl20_adapter import (
    HL20SourceDirectWrenchPlant,
    HL20SourceSurfacePlant,
    build_hl20_source_adapter,
    build_hl20_source_direct_wrench_adapter,
    build_hl20_source_surface_adapter,
)
from .horizontal_fidelity import (
    HorizontalConformanceReport,
    HorizontalFidelityRegistry,
    load_horizontal_registry,
    validate_horizontal_fidelity,
)
from .horizontal_readiness import (
    HorizontalReadinessReport,
    HorizontalShowcasePreflight,
    HorizontalTierReadiness,
    build_horizontal_readiness_report,
    preflight_horizontal_showcase,
)
from .imu_profile_comparison import ImuProfileComparison, compare_imu_profiles
from .mission_objectives import (
    ControllerTransition,
    TruthObjectiveResult,
    TruthObjectiveSpec,
    evaluate_truth_objectives,
)
from .modes import DynamicsMode, FidelitySetupError, Kinematic6DofState, Quaternion
from .navigation import (
    AttitudeNavigationState,
    AttitudeOnlyNavigator,
    DeadReckoningNavigator,
    MekfNoise,
    MultiplicativeEkf,
    NavigationState,
    TranslationNavigationState,
    TranslationOnlyNavigator,
)
from .objectives import ObjectiveResult, ObjectiveSpec, score_objective, score_objectives
from .outputs import DynamicsKind, EventRecord, RunArtifact, SegmentSpan, TelemetryChannel, VehicleKind, VehicleTelemetry, build_run_artifact
from .physical_lqr import (
    PhysicalWrenchLqrDesign,
    PhysicalWrenchLqrSample,
    PhysicalWrenchLqrValidation,
    WrenchLinearizationProjection,
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
    validate_nonlinear_wrench_lqr,
)
from .reachability_catalog import (
    ReachabilityCatalog,
    ReachabilityCommonObject,
    ReachabilityFamilySpec,
    ReachabilityProfileSpec,
    ReachabilityStudySemantic,
    load_reachability_catalog,
)
from .reachability_envelope import (
    DetachedBodyTrajectory,
    EnvelopeBounds,
    EnvelopeSample,
    EnvelopeTermination,
    LaunchCommand,
    PointMass3DofState,
    Pseudo6DofState,
    ReachabilityEnvelope,
    ReachabilityFidelity,
    ReachabilitySearchSpace,
    RigidBody6DofReachabilityState,
    RocketGlideVehicle,
    RocketStageSpec,
    SearchAxis,
    StagedRocketSpec,
    StageSeparationSpec,
    TerminalCriteria,
    TrajectoryResult,
    generate_launch_grid,
    run_reachability_envelope,
    simulate_rocket_glide,
)
from .reachability_visualization import (
    ReachabilityPlotReport,
    load_reachability_artifact,
    plot_children_trajectories,
    plot_deployment_timeline,
    plot_fidelity_progression,
    plot_flown_trajectories,
    plot_parent_trajectory,
    plot_projected_area,
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
from .rigid_body_frames import EarthOperatingPoint, EarthRelativeVelocityStateAdapter, EarthRotationAdapter
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
from .sensors import (
    AccelerationIncrement,
    GyroIncrement,
    IdealGyroscopeAdapter,
    IdealImuAdapter,
    ImuErrorModelAdapter,
    ImuIncrement,
    MeasurementPacket,
    TranslationAccelerationAdapter,
    TruthPoint,
    TruthSegment,
)
from .showcase import (
    ArtifactFile,
    EvidenceBoardSpec,
    FailureCode,
    FamilyShowcaseTemplate,
    FidelityShowcaseRealization,
    MissionSegmentSpec,
    ShowcaseArtifactBoundaryFinding,
    ShowcaseOutcome,
    ShowcaseRunArtifact,
    StartContract,
    TerminalContract,
    VehicleShowcaseBinding,
    build_showcase_run_artifact,
    inspect_showcase_run_artifact,
    validate_showcase_run_artifact_boundary,
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
from .trajectory.evaluation import objective_report_to_evaluation
from .trim import (
    DynamicsLinearization,
    TrimConfigurationError,
    TrimDiagnostic,
    TrimEvaluationError,
    TrimGate,
    TrimGateResult,
    TrimProcedure,
    TrimProcedureResult,
    TrimResult,
    TrimSpec,
    finite_difference_dynamics_linearization,
    finite_difference_linearization,
    solve_trim,
    solve_trim_continuation,
    solve_trim_procedure,
)
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
    DetachedBodyDefinition,
    DetachedBodyShape,
    ImpulseFrame,
    MassProperties,
    PreparedAerodynamicCoefficients,
    PreparedCoefficientTable,
    PropellantType,
    PropulsionCapabilities,
    PropulsionOutput,
    StageDefinition,
    StagedPropulsion,
    StagedVehicleDefinition,
    StageMassDefinition,
    StageSeparationEvent,
    TableAerodynamicModel,
    TumblingPolicy,
)
from .vehicle_controller_mission_preflight import (
    ControllerPreflightResult,
    MissionPreflightResult,
    PreflightFinding,
    VehicleControllerMissionPreflightReport,
    validate_all_vehicle_controller_mission_preflight,
    validate_vehicle_controller_mission_preflight,
)
from .vehicle_effectivity_preflight import (
    EffectivityFinding,
    VehicleEffectivityPreflightReport,
    validate_all_vehicle_effectivity_preflight,
    validate_vehicle_effectivity_preflight,
)
from .vehicle_integration_pipeline import (
    IntegrationPipelineFinding,
    IntegrationStageReport,
    VehicleIntegrationPipelineReport,
    validate_all_vehicle_integration_pipelines,
    validate_vehicle_integration_pipeline,
    write_vehicle_integration_packet,
)
from .vehicle_integration_readiness import (
    FidelityProfileReadiness,
    IntegrationReadinessFinding,
    VehicleIntegrationReadinessReport,
    validate_all_vehicle_integration_readiness,
    validate_vehicle_integration_readiness,
)
from .vehicle_onboarding import (
    OnboardingFinding,
    VehicleOnboardingReport,
    validate_all_vehicle_onboarding,
    validate_vehicle_onboarding,
)
from .vehicle_trim_adapters import (
    TrimSolvePoint,
    VehicleTrimSolveReport,
    solve_all_vehicle_trim_evidence,
    solve_vehicle_trim_evidence,
)
from .vehicle_trim_orchestration import (
    TrimOrchestrationReport,
    TrimRecipe,
    TrimRecipeFinding,
    TrimRecipeResidual,
    TrimRecipeVariable,
    TrimWorkItem,
    load_trim_recipe,
    orchestrate_trim_recipe,
)
from .visualization import render_run_artifact_html, render_run_artifact_plots
from .x15_reachability import (
    X15IntegrationPreflight,
    X15ReachabilityBundle,
    run_x15_reachability_tiers,
    write_x15_reachability_bundle,
    x15_integration_preflight,
    x15_reachability_commands,
    x15_source_staging_contract,
    x15_surrogate_vehicle,
)

__all__ = ["__version__", "AutoTuneCandidate", "AutoTuneLimits", "AutoTuneReport", "auto_tune_lqr_profiles", "default_attitude_linearization", "OnboardingFinding", "VehicleOnboardingReport", "validate_all_vehicle_onboarding", "validate_vehicle_onboarding", "aerodynamics", "atmosphere", "attitude", "contracts", "coordinates", "earth", "equations", "forces", "geodesy", "gravity", "guidance", "iip", "language", "linalg", "modes", "numeric", "objectives", "optimization", "outputs", "radar", "rigid_body", "rigid_body_frames", "rotorcraft", "runtime", "scenario", "segmentation", "searches", "simulation", "state", "state_rates", "table_explorer", "tables", "trim", "validation", "vehicle", "visualization", "AeroQueryContext", "AerodynamicOutput", "AxisInterpolationBracket", "ControlContract", "ControlDirectionProbe", "ControlDirectionResult", "ControllerDesignCatalog", "ControllerDesignMethod", "ControllerDesignSpec", "DebugFamily", "Direction", "DynamicsKind", "DynamicsLinearization", "DynamicsMode", "EarthRotationAdapter", "EventRecord", "FamilyDebugPlan", "GoalSpec", "FamilyDebugRenderReport", "InterpolationExplanation", "Kinematic6DofState", "MassProperties", "ObjectiveResult", "ObjectiveSpec", "OutputContract", "PhaseWindow", "PointMassRates", "PointMassState", "PreparedAerodynamicCoefficients", "PreparedCoefficientTable", "PropulsionOutput", "QuadRotorAllocation", "Quaternion", "RIGID_BODY_STATE_NAMES", "RandomSeed", "ResolutionRecord", "ResolvedScenario", "RigidBody6DofModel", "RigidBody6DofState", "RigidBodyForceMoment", "RotorCommandSet", "RunArtifact", "ScenarioCompileError", "ScenarioCompiler", "ScenarioRequest", "ScenarioRuntimeContract", "ScenarioSource", "SegmentSpan", "SegmentSpec", "SegmentationCatalog", "SegmentationScenario", "StageDefinition", "StagedPropulsion", "StatusContract", "TableAerodynamicModel", "TableInspection", "TableInspectionArtifact", "TableInspectionFormat", "TableInspectionStatus", "TelemetryChannel", "ThermalAssessment", "ThermalLimits", "TransitionEventSpec", "TransitionPolicy", "TrimCatalog", "TrimCatalogEntry", "TrimResult", "TrimSpec", "VehicleKind", "VehicleTelemetry", "actuator_saturation_fraction", "assess_thermal_limits", "audit_control_directions", "build_family_debug_plan", "build_lqr_controller", "build_run_artifact", "capture_time", "dwell_in_band", "energy_balance_residual", "explain_interpolation", "family_profile", "finite_difference_dynamics_linearization", "finite_difference_linearization", "independent_force_closure", "inspect_table_document", "inspect_table_file", "integral_mass_balance_error", "load_controller_catalog", "load_trim_catalog", "phase_slice", "render_family_debug_artifacts", "render_run_artifact_html", "render_run_artifact_plots", "require_bounded", "require_change_of_sign", "require_channel", "require_monotonic", "require_net_change", "score_objective", "score_objectives", "require_net_change", "settling_time", "solve_trim", "specific_energy", "timestep_convergence_error", "transition_audit", "wrapped_angle_error"]
__all__ += ["TrimGate", "TrimGateResult", "TrimProcedure", "TrimProcedureResult", "solve_trim_continuation", "solve_trim_procedure", "objective_report_to_evaluation"]
__all__ += ["showcase", "ArtifactFile", "EvidenceBoardSpec", "FailureCode", "FidelityShowcaseRealization", "ShowcaseArtifactBoundaryFinding", "build_showcase_run_artifact", "inspect_showcase_run_artifact", "validate_showcase_run_artifact_boundary", "FamilyShowcaseTemplate", "MissionSegmentSpec", "ShowcaseRunArtifact", "ShowcaseOutcome", "StartContract", "TerminalContract", "VehicleShowcaseBinding"]
__all__ = ["__version__", "AutoTuneCandidate", "AutoTuneLimits", "AutoTuneReport", "ReachabilityCatalog", "ReachabilityCommonObject", "ReachabilityFamilySpec", "ReachabilityProfileSpec", "ReachabilityStudySemantic", "DetachedBodyTrajectory", "EnvelopeBounds", "EnvelopeSample", "EnvelopeTermination", "LaunchCommand", "PointMass3DofState", "Pseudo6DofState", "RigidBody6DofReachabilityState", "ReachabilityEnvelope", "ReachabilityFidelity", "ReachabilitySearchSpace", "RocketGlideVehicle", "RocketStageSpec", "SearchAxis", "StageSeparationSpec", "StagedRocketSpec", "TerminalCriteria", "TrajectoryResult", "ReachabilityPlotReport", "load_reachability_artifact", "plot_children_trajectories", "plot_deployment_timeline", "plot_fidelity_progression", "plot_flown_trajectories", "plot_parent_trajectory", "plot_projected_area", "plot_search_coverage", "plot_terminal_capability", "render_reachability_plot_bundle", "X15IntegrationPreflight", "X15ReachabilityBundle", "x15_integration_preflight", "run_x15_reachability_tiers", "write_x15_reachability_bundle", "x15_reachability_commands", "x15_source_staging_contract", "x15_surrogate_vehicle", "generate_launch_grid", "run_reachability_envelope", "simulate_rocket_glide", "auto_tune_lqr_profiles", "default_attitude_linearization", "OnboardingFinding", "VehicleOnboardingReport", "load_reachability_catalog", "validate_all_vehicle_onboarding", "validate_vehicle_onboarding", "aerodynamics", "atmosphere", "attitude", "contracts", "coordinates", "earth", "equations", "forces", "geodesy", "gravity", "guidance", "iip", "language", "linalg", "modes", "numeric", "objectives", "optimization", "outputs", "radar", "reachability_catalog", "reachability_envelope", "reachability_visualization", "x15_reachability", "rigid_body", "rigid_body_frames", "rotorcraft", "runtime", "scenario", "segmentation", "searches", "simulation", "state", "state_rates", "table_explorer", "tables", "trim", "validation", "vehicle", "visualization", "AeroQueryContext", "AerodynamicOutput", "AxisInterpolationBracket", "ControlContract", "ControlDirectionProbe", "ControlDirectionResult", "ControllerDesignCatalog", "ControllerDesignMethod", "ControllerDesignSpec", "DebugFamily", "DetachedBodyDefinition", "DetachedBodyShape", "Direction", "DynamicsKind", "DynamicsLinearization", "DynamicsMode", "EarthRotationAdapter", "EventRecord", "FamilyDebugPlan", "GoalSpec", "FamilyDebugRenderReport", "ImpulseFrame", "InterpolationExplanation", "Kinematic6DofState", "MassProperties", "ObjectiveResult", "ObjectiveSpec", "OutputContract", "PhaseWindow", "PointMassRates", "PointMassState", "PreparedAerodynamicCoefficients", "PreparedCoefficientTable", "PropulsionOutput", "PropellantType", "PropulsionCapabilities", "StageMassDefinition", "StageDefinition", "StageSeparationEvent", "StagedVehicleDefinition", "StagedPropulsion", "QuadRotorAllocation", "Quaternion", "RIGID_BODY_STATE_NAMES", "RandomSeed", "ResolutionRecord", "ResolvedScenario", "RigidBody6DofModel", "RigidBody6DofState", "RigidBodyForceMoment", "RotorCommandSet", "RunArtifact", "ScenarioCompileError", "ScenarioCompiler", "ScenarioRequest", "ScenarioRuntimeContract", "ScenarioSource", "SegmentSpan", "SegmentSpec", "SegmentationCatalog", "SegmentationScenario", "StatusContract", "TableAerodynamicModel", "TableInspection", "TableInspectionArtifact", "TableInspectionFormat", "TableInspectionStatus", "TelemetryChannel", "ThermalAssessment", "ThermalLimits", "TransitionEventSpec", "TransitionPolicy", "TrimCatalog", "TrimCatalogEntry", "TrimResult", "TrimSpec", "TumblingPolicy", "VehicleKind", "VehicleTelemetry", "actuator_saturation_fraction", "assess_thermal_limits", "audit_control_directions", "build_family_debug_plan", "build_lqr_controller", "build_run_artifact", "capture_time", "dwell_in_band", "energy_balance_residual", "explain_interpolation", "family_profile", "finite_difference_dynamics_linearization", "finite_difference_linearization", "independent_force_closure", "inspect_table_document", "inspect_table_file", "integral_mass_balance_error", "load_controller_catalog", "load_trim_catalog", "phase_slice", "render_family_debug_artifacts", "render_run_artifact_html", "render_run_artifact_plots", "require_bounded", "require_change_of_sign", "require_channel", "require_monotonic", "require_net_change", "score_objective", "score_objectives", "require_net_change", "settling_time", "solve_trim", "specific_energy", "timestep_convergence_error", "transition_audit", "wrapped_angle_error"]
__all__ += ["TrimGate", "TrimGateResult", "TrimProcedure", "TrimProcedureResult", "solve_trim_continuation", "solve_trim_procedure", "objective_report_to_evaluation"]
__all__ += ["showcase", "ArtifactFile", "EvidenceBoardSpec", "FailureCode", "FidelityShowcaseRealization", "ShowcaseArtifactBoundaryFinding", "build_showcase_run_artifact", "inspect_showcase_run_artifact", "validate_showcase_run_artifact_boundary", "FamilyShowcaseTemplate", "MissionSegmentSpec", "ShowcaseRunArtifact", "ShowcaseOutcome", "StartContract", "TerminalContract", "VehicleShowcaseBinding"]
__all__ += ["ControllerTransition", "TruthObjectiveResult", "TruthObjectiveSpec", "evaluate_truth_objectives"]
__all__ += [
    "DIRECT_WRENCH_NAMES",
    "DirectWrenchLimits",
    "DirectWrenchProjection",
    "add_direct_wrench_to_local_derivative",
    "compose_direct_wrench_load",
]
__all__ += [
    "GenericLqrCandidate",
    "GenericLqrProfile",
    "AuthorityPreflightReport",
    "LinearAuthorityRequirement",
    "GenericLqrReport",
    "TrimToTuneResult",
    "linear_authority_preflight",
    "linear_authority_preflight_evaluator",
    "trim_linearize_and_tune",
    "tune_lqr_profiles",
]
__all__ += [
    "PhysicalWrenchLqrDesign",
    "PhysicalWrenchLqrSample",
    "PhysicalWrenchLqrValidation",
    "WrenchLinearizationProjection",
    "design_physical_wrench_lqr",
    "project_linearization_to_wrench",
    "validate_nonlinear_wrench_lqr",
]
__all__ += [
    "AllocationResult",
    "ClosedLoopPole",
    "ControllerChannel",
    "ControllerInventory",
    "ControllerInventoryEntry",
    "ControllerBackendFactory",
    "ControllerBackendRegistry",
    "ControllerBackendSpec",
    "ControllerBackendStatus",
    "ControllerProvenance",
    "ControllerQualificationStatus",
    "ControllerPreflightIssue",
    "ControllerPreflightReport",
    "ControllerRealization",
    "ControllerRuntimeState",
    "ControllerSchedule",
    "GeneralizedControlRequest",
    "GuidanceReference",
    "default_controller_backend_registry",
    "preflight_controller_realization",
]
__all__ += ["ImuProfileComparison", "compare_imu_profiles"]
__all__ += [
    "EarthOperatingPoint",
    "EarthRelativeVelocityStateAdapter",
    "FidelitySetupError",
    "TrimConfigurationError",
    "TrimDiagnostic",
    "TrimEvaluationError",
]
__all__ += [
    "AttitudeNavigationState",
    "AttitudeOnlyNavigator",
    "DeadReckoningNavigator",
    "TranslationNavigationState",
    "TranslationOnlyNavigator",
    "AccelerationIncrement",
    "GyroIncrement",
    "IdealGyroscopeAdapter",
    "IdealImuAdapter",
    "ImuErrorModelAdapter",
    "ImuIncrement",
    "MeasurementPacket",
    "MekfNoise",
    "MultiplicativeEkf",
    "NavigationState",
    "TruthPoint",
    "TruthSegment",
    "TranslationAccelerationAdapter",
]
__all__ += [
    "EffectivityFinding",
    "VehicleEffectivityPreflightReport",
    "validate_all_vehicle_effectivity_preflight",
    "validate_vehicle_effectivity_preflight",
]
__all__ += [
    "FidelityProfileReadiness",
    "IntegrationReadinessFinding",
    "VehicleIntegrationReadinessReport",
    "validate_all_vehicle_integration_readiness",
    "validate_vehicle_integration_readiness",
    "IntegrationPipelineFinding",
    "IntegrationStageReport",
    "VehicleIntegrationPipelineReport",
    "validate_all_vehicle_integration_pipelines",
    "validate_vehicle_integration_pipeline",
    "write_vehicle_integration_packet",
]
__all__ += [
    "TrimOrchestrationReport",
    "TrimRecipe",
    "TrimRecipeFinding",
    "TrimRecipeResidual",
    "TrimRecipeVariable",
    "TrimWorkItem",
    "load_trim_recipe",
    "orchestrate_trim_recipe",
]
__all__ += [
    "ControllerPreflightResult",
    "MissionPreflightResult",
    "PreflightFinding",
    "VehicleControllerMissionPreflightReport",
    "validate_all_vehicle_controller_mission_preflight",
    "validate_vehicle_controller_mission_preflight",
]
__all__ += [
    "TrimSolvePoint",
    "VehicleTrimSolveReport",
    "solve_all_vehicle_trim_evidence",
    "solve_vehicle_trim_evidence",
]
__all__ += [
    "CANONICAL_FIDELITY_TIERS",
    "FIDELITY_TIER_RANK",
    "LEGACY_FIDELITY_ORDER",
    "canonicalize_fidelity",
    "canonical_tier_for_runtime",
    "control_realization_for",
    "parent_fidelity",
    "runtime_fidelity_for",
    "LoweringCandidate",
    "LoweringDecision",
    "select_canonical_lowering",
    "HorizontalConformanceReport",
    "HorizontalFidelityRegistry",
    "load_horizontal_registry",
    "validate_horizontal_fidelity",
    "HL20SourceDirectWrenchPlant",
    "HL20SourceSurfacePlant",
    "build_hl20_source_adapter",
    "build_hl20_source_direct_wrench_adapter",
    "build_hl20_source_surface_adapter",
    "AdapterCapability",
    "AdapterCapabilityError",
    "AdapterChannel",
    "AdapterConformanceFinding",
    "AdapterConformanceReport",
    "FamilyAdapter",
    "FamilyAdapterDescriptor",
    "FamilyCapabilityReport",
    "AllocationProvider",
    "EffectivenessProvider",
    "StandardFamilyAdapter",
    "StateDerivativeProvider",
    "descriptor_from_control_plant",
    "descriptor_from_direct_wrench_state",
    "validate_family_adapter",
    "AdapterProbeCase",
    "AdapterProbeOperation",
    "AdapterProbeReport",
    "run_adapter_probe",
    "AdapterRegistrationCheck",
    "AdapterRegistrationError",
    "AdapterRegistryReport",
    "FamilyAdapterProbeFactory",
    "FamilyAdapterRegistration",
    "FamilyAdapterRegistry",
    "UnifiedFamilyManifest",
    "UnifiedFamilyManifestCatalog",
    "UnifiedFamilyManifestFinding",
    "load_unified_family_manifest_catalog",
    "FAMILY_STRATEGY_CATALOG",
    "FamilyIntegrationStrategy",
    "FamilyIntegrationStrategyCatalog",
    "FamilyStrategyConformanceReport",
    "FamilyStrategyWorkItem",
    "FamilyStrategyWorklistReport",
    "FamilyTierStrategy",
    "AuthorityPreflightReport",
    "build_family_strategy_worklist",
    "load_family_strategy_catalog",
    "validate_family_strategy_catalog",
    "HorizontalReadinessReport",
    "HorizontalShowcasePreflight",
    "HorizontalTierReadiness",
    "build_horizontal_readiness_report",
    "preflight_horizontal_showcase",
]
__all__ += [
    "composition_episode",
    "composition_policy",
    "EpisodeChannel",
    "EpisodeObservation",
    "EpisodeStep",
    "HummingbirdPseudoCompositionEpisode",
    "LanguageBackedCompositionEpisode",
    "VehicleCompositionEpisode",
    "open_vehicle_composition_episode",
    "CompositionPolicyTrace",
    "PolicyDecision",
    "PolicyFunction",
    "run_composition_policy",
]
