"""Generate explicit intake blueprints for existing and new vehicle families.

The horizontal strategy catalog should make an ordinary vehicle addition a
data-and-adapter task, not an invitation to start a bespoke controller.  This
module turns that catalog into an actionable four-tier blueprint before any
family manifest or source mapping is written.

It deliberately does not infer a strategy when multiple topologies share a
physical-family label.  A rocket aircraft and an air-breathing fixed wing are
both powered fixed wing, but their phase boundaries and authority checks are
not interchangeable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Mapping, get_args

from .family_strategy import FamilyIntegrationStrategy, FamilyIntegrationStrategyCatalog, load_family_strategy_catalog
from .fidelity_contracts import CANONICAL_FIDELITY_TIERS, FidelityTier, control_realization_for
from .vehicle_composition_registry import ParameterSpec
from .vehicle_execution_bindings import ExecutionMode

IntakeKind = Literal["existing_family", "new_topology"]


class StrategySelectionError(ValueError):
    """Raised when an intake cannot select one compatible strategy safely."""


@dataclass(frozen=True, slots=True)
class ExistingFamilyIntakeRequest:
    """The minimal identity needed to add a vehicle to an existing topology."""

    family_id: str
    physical_family: str
    mission_overlay: str
    adapter_id: str
    strategy_id: str | None = None

    def __post_init__(self) -> None:
        values = (self.family_id, self.physical_family, self.mission_overlay, self.adapter_id)
        if not all(value.strip() for value in values):
            raise ValueError("existing-family intake requires non-empty family, topology, mission, and adapter IDs")
        ####

    ####


@dataclass(frozen=True, slots=True)
class IntakeTierBlueprint:
    """One generated canonical fidelity slot and its required evidence path."""

    tier: FidelityTier
    control_realization: str
    data_requirements: tuple[str, ...]
    required_operations: tuple[str, ...]
    calibration_mode: str
    stages: tuple[str, ...]
    operating_point_campaign_required: bool
    initial_blockers: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a portable tier-level intake contract."""

        return {
            "tier": self.tier,
            "control_realization": self.control_realization,
            "data_requirements": list(self.data_requirements),
            "required_operations": list(self.required_operations),
            "calibration_mode": self.calibration_mode,
            "stages": list(self.stages),
            "operating_point_campaign_required": self.operating_point_campaign_required,
            "initial_blockers": list(self.initial_blockers),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class ExistingFamilyIntakeBlueprint:
    """Generated declaration and work package for an existing family strategy."""

    request: ExistingFamilyIntakeRequest
    strategy: FamilyIntegrationStrategy
    tiers: tuple[IntakeTierBlueprint, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a registry-entry template plus the unambiguous worklist."""

        registry_tiers = {
            tier.tier: {
                "profile_id": None,
                "promotion_status": "planned",
                "required_operations": list(tier.required_operations),
                "blockers": list(tier.initial_blockers),
            }
            for tier in self.tiers
        }
        payload: dict[str, object] = {
            "schema": "taoryx.vehicle-integration-intake/v1alpha1",
            "kind": "existing_family",
            "status": "ready_for_source_and_adapter_binding",
            "family_id": self.request.family_id,
            "physical_family": self.request.physical_family,
            "mission_overlay": self.request.mission_overlay,
            "strategy_id": self.strategy.id,
            "adapter_id": self.request.adapter_id,
            "family_inputs": list(self.strategy.family_inputs),
            "non_tunable_blockers": list(self.strategy.non_tunable_blockers),
            "interface_contract_required": _interface_contract_blueprint(),
            "mission_binding_contract_required": _mission_binding_contract_blueprint(),
            "mission_composition": _mission_composition_blueprint(self.strategy, self.request),
            "synthetic_conformance_witness_plan": _synthetic_conformance_witness_plan(),
            "registry_entry_template": {
                "family_id": self.request.family_id,
                "physical_family": self.request.physical_family,
                "strategy_id": self.strategy.id,
                "mission_overlay": self.request.mission_overlay,
                "adapter_id": self.request.adapter_id,
                "automatic_lowering": True,
                "tiers": registry_tiers,
                "interface_contract_status": "required_before_execution_binding",
            },
            "tiers": [tier.as_dict() for tier in self.tiers],
            "claim_boundary": (
                "Intake blueprint only. Profile IDs, source evidence, plant operations, "
                "and qualification results remain undeclared until independently supplied."
            ),
        }
        return _with_intake_validation(payload)
        ####

    ####


@dataclass(frozen=True, slots=True)
class NewTopologyIntakeRequest:
    """The information known before a genuinely new physical family exists."""

    family_id: str
    physical_family: str
    proposed_strategy_id: str
    topology_summary: str

    def __post_init__(self) -> None:
        values = (self.family_id, self.physical_family, self.proposed_strategy_id, self.topology_summary)
        if not all(value.strip() for value in values):
            raise ValueError("new-topology intake requires non-empty identity and topology summary fields")
        ####

    ####


@dataclass(frozen=True, slots=True)
class NewTopologyIntakeScaffold:
    """The required decisions for a new topology without fabricated physics."""

    request: NewTopologyIntakeRequest

    def as_dict(self) -> dict[str, object]:
        """Return a strategy-authoring checklist and non-promotable template."""

        tiers = [
            {
                "tier": tier,
                "control_realization": control_realization_for(tier),
                "required_decisions": [
                    "applicability for this topology",
                    "minimum source/data contract",
                    "actual adapter operations and their evidence",
                    "calibration mode and ordered diagnostic stages",
                    "claim boundary and non-tunable topology blockers",
                ],
            }
            for tier in CANONICAL_FIDELITY_TIERS
        ]
        payload: dict[str, object] = {
            "schema": "taoryx.vehicle-integration-intake/v1alpha1",
            "kind": "new_topology",
            "status": "strategy_definition_required",
            "family_id": self.request.family_id,
            "physical_family": self.request.physical_family,
            "proposed_strategy_id": self.request.proposed_strategy_id,
            "topology_summary": self.request.topology_summary,
            "required_strategy_fields": [
                "summary",
                "family_inputs",
                "non_tunable_blockers",
                "exactly four canonical tier recipes",
                "Mission Composition interface-contract declaration",
            ],
            "source_manifest_template": _source_manifest_template(self.request),
            "new_family_decision_record": _new_family_decision_record(self.request),
            "interface_contract_required": _interface_contract_blueprint(),
            "mission_binding_contract_required": _mission_binding_contract_blueprint(),
            "synthetic_conformance_witness_plan": _synthetic_conformance_witness_plan(),
            "tiers": tiers,
            "claim_boundary": (
                "A new topology scaffold is not a vehicle model. No data, operation, "
                "or control claim is inferred until the proposed strategy is authored and validated."
            ),
        }
        return _with_intake_validation(payload)
        ####

    ####


def compatible_strategies(
    physical_family: str,
    *,
    catalog: FamilyIntegrationStrategyCatalog | None = None,
) -> tuple[FamilyIntegrationStrategy, ...]:
    """Return catalog strategies compatible with one physical-family label."""

    resolved_catalog = catalog or load_family_strategy_catalog()
    return tuple(strategy for strategy in resolved_catalog.strategies if physical_family in strategy.physical_families)
    ####


def select_existing_family_strategy(
    request: ExistingFamilyIntakeRequest,
    *,
    catalog: FamilyIntegrationStrategyCatalog | None = None,
) -> FamilyIntegrationStrategy:
    """Select a compatible strategy, refusing ambiguous topology inference."""

    resolved_catalog = catalog or load_family_strategy_catalog()
    compatible = compatible_strategies(request.physical_family, catalog=resolved_catalog)
    if request.strategy_id is not None:
        strategy = resolved_catalog.strategy(request.strategy_id)
        if request.physical_family not in strategy.physical_families:
            raise StrategySelectionError(f"strategy {strategy.id!r} does not support physical family {request.physical_family!r}")
        return strategy
    if not compatible:
        raise StrategySelectionError(f"no registered strategy supports physical family {request.physical_family!r}; create a new topology strategy")
    if len(compatible) != 1:
        candidates = ", ".join(strategy.id for strategy in compatible)
        raise StrategySelectionError(f"physical family {request.physical_family!r} is ambiguous across strategies: {candidates}; select strategy_id explicitly")
    return compatible[0]
    ####


def build_existing_family_intake_blueprint(
    request: ExistingFamilyIntakeRequest,
    *,
    catalog: FamilyIntegrationStrategyCatalog | None = None,
) -> ExistingFamilyIntakeBlueprint:
    """Generate a complete four-tier intake work package for one new vehicle."""

    strategy = select_existing_family_strategy(request, catalog=catalog)
    tiers: list[IntakeTierBlueprint] = []
    for tier in CANONICAL_FIDELITY_TIERS:
        recipe = strategy.tier_recipes[tier]
        initial_blockers = (
            (
                "source_lock_and_frame_contract_missing",
                "profile_and_adapter_binding_missing",
                *strategy.non_tunable_blockers,
            )
            if recipe.applicable
            else ("not_applicable_by_declared_topology",)
        )
        tiers.append(
            IntakeTierBlueprint(
                tier,
                control_realization_for(tier),
                recipe.data,
                tuple(recipe.operations),
                recipe.calibration_mode,
                recipe.stages,
                "operating_point_campaign" in recipe.stages,
                initial_blockers,
            )
        )
    return ExistingFamilyIntakeBlueprint(request, strategy, tuple(tiers))
    ####


def build_new_topology_intake_scaffold(request: NewTopologyIntakeRequest) -> NewTopologyIntakeScaffold:
    """Return the explicit authoring checklist for a new physical topology."""

    return NewTopologyIntakeScaffold(request)
    ####


def _mission_composition_blueprint(
    strategy: FamilyIntegrationStrategy,
    request: ExistingFamilyIntakeRequest,
) -> dict[str, object]:
    """Return mission-layer inputs without pretending they are vehicle data."""

    if strategy.id == "powered_fixed_wing.v1" and request.mission_overlay == "fixed_wing_racetrack":
        return {
            "template_id": "powered_fixed_wing_racetrack_v1",
            "capability_profile_required": [
                "operating_point_id",
                "minimum_speed_m_s",
                "nominal_speed_m_s",
                "maximum_speed_m_s",
                "maximum_bank_deg",
                "maximum_climb_rate_m_s",
                "maximum_descent_rate_m_s",
                "provenance",
                "evidence_class",
            ],
            "semantic_intent_required": [
                "low_altitude_m",
                "high_altitude_m",
                "requested_speed_m_s",
                "requested_bank_deg",
                "minimum_straight_length_m",
                "level_dwell_s",
            ],
            "capability_profile_parameter_contracts": _powered_fixed_wing_capability_parameter_contracts(),
            "semantic_intent_parameter_contracts": _powered_fixed_wing_racetrack_parameter_contracts(),
            "compiler": "taoryx.powered_fixed_wing_mission_compiler.compile_powered_fixed_wing_racetrack",
            "expected_artifacts": [
                "capability_scaled_route_manifest",
                "baseline_comparison",
                "truth_objective_report",
            ],
            "claim_boundary": (
                "Compiled route geometry is a planning/preflight result. It does not prove "
                "controller tracking, nonlinear response, or physical effector allocation."
            ),
        }
    return {
        "template_id": request.mission_overlay,
        "status": "family_specific_mission_compiler_required",
        "claim_boundary": "No mission geometry or controller requirement is inferred for this overlay.",
    }
    ####


def _intake_parameter_contract(parameter: ParameterSpec, *, scope: str) -> dict[str, object]:
    """Serialize a typed unfilled input contract for an intake scaffold.

    These records use exactly the same public descriptor as the composition
    registry.  They deliberately declare only mathematical validity available
    from the shared mission semantics; source-backed defaults, qualified
    limits, and provenance remain absent until the new family supplies them.
    """

    payload = parameter.public_dict()
    payload["scope"] = scope
    payload["source_value_status"] = "authoring_required"
    payload["claim_boundary"] = "This is an intake parameter contract, not a vehicle-specific source value, qualified limit, or runtime binding."
    return payload
    ####


def _powered_fixed_wing_capability_parameter_contracts() -> list[dict[str, object]]:
    """Return the typed source-owned inputs required by the shared racetrack planner."""

    parameters = (
        ParameterSpec(
            id="operating_point_id",
            value_type_declared="enum",
            required=True,
            description="Stable source-defined operating-point identity for the capability profile.",
        ),
        ParameterSpec(
            id="minimum_speed_m_s",
            canonical_unit="m/s",
            required=True,
            hard_lower=0.0,
            description="Minimum source-supported true airspeed at the selected operating point.",
        ),
        ParameterSpec(
            id="nominal_speed_m_s",
            canonical_unit="m/s",
            required=True,
            hard_lower=0.0,
            description="Nominal route speed used to size the first-pass geometry.",
        ),
        ParameterSpec(
            id="maximum_speed_m_s",
            canonical_unit="m/s",
            required=True,
            hard_lower=0.0,
            description="Maximum source-supported speed for the selected operating point.",
        ),
        ParameterSpec(
            id="maximum_bank_deg",
            canonical_unit="deg",
            required=True,
            hard_lower=0.0,
            hard_upper=89.0,
            description="Maximum bank magnitude admitted by the declared profile.",
        ),
        ParameterSpec(
            id="maximum_climb_rate_m_s",
            canonical_unit="m/s",
            required=True,
            hard_lower=0.0,
            description="Maximum sustained positive climb rate for first-pass route sizing.",
        ),
        ParameterSpec(
            id="maximum_descent_rate_m_s",
            canonical_unit="m/s",
            required=True,
            hard_lower=0.0,
            description="Maximum sustained descent-rate magnitude for first-pass route sizing.",
        ),
    )
    return [_intake_parameter_contract(parameter, scope="family_capability_profile") for parameter in parameters]
    ####


def _powered_fixed_wing_racetrack_parameter_contracts() -> list[dict[str, object]]:
    """Return the typed mission variables reusable by a fixed-wing member."""

    parameters = (
        ParameterSpec(
            id="low_altitude_m",
            canonical_unit="m",
            required=True,
            hard_lower=0.0,
            description="Lower level-flight altitude for the racetrack vertical profile.",
        ),
        ParameterSpec(
            id="high_altitude_m",
            canonical_unit="m",
            required=True,
            hard_lower=0.0,
            description="Upper level-flight altitude for the racetrack vertical profile.",
        ),
        ParameterSpec(
            id="requested_speed_m_s",
            canonical_unit="m/s",
            required=True,
            hard_lower=0.0,
            description="Requested nominal speed, checked by the family capability adapter.",
        ),
        ParameterSpec(
            id="requested_bank_deg",
            canonical_unit="deg",
            required=True,
            hard_lower=0.0,
            hard_upper=89.0,
            description="Requested bank magnitude, checked against the family capability profile.",
        ),
        ParameterSpec(
            id="turn_direction",
            value_type_declared="enum",
            required=True,
            description="Declared initial turn direction; a complete racetrack must exercise both signs.",
        ),
        ParameterSpec(
            id="minimum_straight_length_m",
            canonical_unit="m",
            required=True,
            hard_lower=0.0,
            description="Requested minimum settled straight-leg length before the next maneuver.",
        ),
        ParameterSpec(
            id="level_dwell_s",
            canonical_unit="s",
            required=True,
            hard_lower=0.0,
            description="Requested level-flight dwell time for separated vertical and lateral exercises.",
        ),
    )
    return [_intake_parameter_contract(parameter, scope="segment") for parameter in parameters]
    ####


def _interface_contract_blueprint() -> dict[str, object]:
    """Return the minimum composability declaration for an intake package.

    A source manifest can describe a plant without telling a user, policy, or
    mission author what is safe to configure or observe. This checklist makes
    that Mission Composition boundary explicit before an author begins inventing a
    vehicle-specific wrapper. It contains declaration work only; no item is
    interpreted as a supplied model property or a fidelity promotion.
    """

    return {
        "status": "required_before_execution_binding",
        "parameter_contract": {
            "required_scopes": [
                "variant_configuration",
                "episode_reset",
                "segment",
                "derived_status",
            ],
            "required_per_parameter": [
                "stable_semantic_id",
                "canonical_unit_and_frame",
                "value_type",
                "value_space_topology_and_representation",
                "hard_and_qualified_bounds",
                "mutability_scope",
                "provenance",
                "runtime_binding_or_derived_formula",
                "retrim_reschedule_and_requalification_invalidations",
            ],
        },
        "control_contract": {
            "required_authority_layers": [
                "mission_or_kinematic_intent_when_applicable",
                "body_motion_or_wrench_when_applicable",
                "physical_effector_or_explicit_bridge_boundary",
            ],
            "required_per_action_or_effector": [
                "stable_semantic_id",
                "canonical_unit_and_frame",
                "value_space_topology_and_bounds",
                "availability_by_fidelity",
                "native_or_allocator_binding",
                "commanded_vs_achieved_telemetry",
                "claim_boundary",
            ],
        },
        "status_observation_resource_contract": {
            "required_canonical_status_domains": [
                "execution_time_and_status",
                "position_velocity_and_attitude_when_represented",
                "propulsion_or_energy_status_when_represented",
                "resource_ledger_when_represented",
                "control_realization_and_saturation",
                "envelope_and_numerical_status",
            ],
            "required_per_channel": [
                "canonical_unit_and_frame",
                "value_space_topology_and_representation",
                "truth_or_sensor_sampling_semantics",
                "availability",
                "source_or_runtime_binding",
                "evidence_boundary",
            ],
        },
        "execution_and_evaluation_contract": [
            "exact_batch_and_episode_factory_or_declared_gap",
            "execution_mode_and_batch_episode_disposition",
            "batch_action_trace_disposition",
            "committed_truth_boundary_and_sensor_cadence_policy",
            "mission_objective_and_terminal_semantics",
            "normalized_result_and_provenance_artifacts",
            "replay_or_parity_witness_plan_when_both_modes_are_advertised",
        ],
        "execution_mode_contract": {
            "status": "required_before_execution_binding",
            "allowed_values": list(get_args(ExecutionMode)),
            "required_per_endpoint": [
                "execution_mode",
                "batch_or_episode_operation",
                "batch_action_trace_disposition",
                "factory_id_or_explicit_gap",
                "claim_boundary",
            ],
            "constraints": [
                "source_history_replay_source_scheduled_replay_and_passive_uncontrolled_are_batch_only",
                "local_direct_wrench_screen_requires_rigid_body_6dof_direct_wrench",
                "source_surface_authority_screen_is_batch_only",
                "planned_status_requires_execution_mode_planned",
            ],
        },
        "claim_boundary": (
            "These are required declarations for a composable vehicle. They do not supply missing source data, "
            "create controls, bind an adapter, or qualify a selected fidelity."
        ),
    }
    ####


def _mission_binding_contract_blueprint() -> dict[str, object]:
    """Return the shared source-family-to-composition mission-binding contract.

    A family integration cannot assume that its flagship is a racetrack.  The
    legacy racetrack record stays available for existing fixed-wing packages;
    new authoring can instead pin a semantic composition witness that proves
    only compile/preflight identity.  Both forms preserve the separation
    between a mission's semantic contract and a native runtime promotion.
    """

    return {
        "status": "required_before_mission_preflight",
        "supported_kinds": ["legacy_racetrack", "semantic_composition"],
        "semantic_composition": {
            "required_fields": [
                "schema: taoryx.mission-binding/v1alpha1",
                "kind: semantic_composition",
                "family: source-family identifier",
                "composition_family_id: public composition identifier",
                "mission_id: public mission-template identifier",
                "phase_order: exact ordered segment IDs",
                "realizations: one or more checked-in composition witnesses",
                "status and claim_boundary",
            ],
            "required_per_realization": [
                "fidelity",
                "composition: repository-relative request path",
                "expected_preflight_status: translation_ready | blocked | not_applicable",
            ],
            "validation": [
                "compile witness through the public composition catalog",
                "match source family, public composition family, mission, and fidelity exactly",
                "match declared phase order exactly",
                "match declared semantic-preflight disposition exactly",
                "retain runtime/evaluation/qualification as independent promotion gates",
            ],
        },
        "legacy_racetrack": {
            "required_fields": [
                "shared racetrack template/source catalog",
                "vehicle-specific realization IDs",
                "ordered phase contract",
                "capability-scaled duration inputs",
            ],
            "claim_boundary": "A legacy racetrack binding is not a privileged universal mission form; it remains one reusable semantic binding kind.",
        },
        "claim_boundary": (
            "A mission binding proves only that a source family selects a declared semantic mission and its retained "
            "preflight witnesses. It never creates a native runtime, controller, truth-objective result, or qualification."
        ),
    }
    ####


def _source_manifest_template(request: NewTopologyIntakeRequest) -> dict[str, object]:
    """Return an intentionally unfilled source-manifest authoring shape."""

    return {
        "schema": "taoryx.family-source-manifest/v1alpha1",
        "family_id": request.family_id,
        "physical_family": request.physical_family,
        "status": "authoring_required",
        "required_fields": [
            "display_name",
            "source_assets_with_sha256",
            "license_or_release_basis",
            "evidence_class_by_subsystem",
            "units_frames_and_sign_conventions",
            "model_assumptions_and_nonclaims",
        ],
        "authoring_values": None,
        "claim_boundary": (
            "This is a source-manifest shape only. It contains no source asset, hash, release assertion, "
            "or evidence classification until an author supplies and validates those records."
        ),
    }
    ####


def _new_family_decision_record(request: NewTopologyIntakeRequest) -> dict[str, object]:
    """Return the auditable decision record required before catalog admission."""

    return {
        "schema": "taoryx.new-family-decision-record/v1alpha1",
        "status": "decision_required",
        "family_id": request.family_id,
        "physical_family": request.physical_family,
        "proposed_strategy_id": request.proposed_strategy_id,
        "topology_summary": request.topology_summary,
        "required_decisions": [
            "why no existing family strategy is suitable",
            "component topology and independent configuration parameters",
            "mass_resource_and_energy_coupling",
            "start_contracts_and_terminal_contracts",
            "applicable fidelity tiers and explicit non-applicable tiers",
            "control_authority_effector_or_uncontrolled_boundary",
            "truth_observation_and_sensor_timing contract",
            "mission/objective and failure taxonomy mapping",
            "source_provenance_and_evidence_classification",
            "qualification and nonclaim boundary",
        ],
        "approved_by": None,
        "decision_date": None,
        "claim_boundary": (
            "No topology decision has been made. This record prevents a new family from being admitted to the "
            "composition registry as an undocumented variation of an existing model."
        ),
    }
    ####


def _synthetic_conformance_witness_plan() -> list[dict[str, object]]:
    """Declare generic test intents without inventing a family dynamics fixture."""

    return [
        {
            "tier": tier,
            "control_realization": control_realization_for(tier),
            "required_witnesses": [
                "interface_descriptor_and_value_space_validation",
                "initialization_and_parameter_scope_rejection",
                "status_observation_projection_at_committed_truth_boundaries",
                "mission_graph_compilation_and_unsupported-transition rejection",
                "execution_binding_or_explicit_unavailable record",
                "execution_mode_and_batch_action_trace_disposition validation",
            ],
        }
        for tier in CANONICAL_FIDELITY_TIERS
    ]
    ####


def validate_integration_intake_payload(payload: Mapping[str, object]) -> tuple[str, ...]:
    """Return structural errors for one generated Mission Composition intake document.

    Intake is intentionally non-promotable, but it must still be safe to hand
    to an integration author or an automation system.  This check verifies the
    common, no-invented-data part of the contract: all four canonical fidelity
    decisions, their declared realization boundaries, the interface checklist,
    and a witness plan.  It never tries to validate source data, plant physics,
    or an author-supplied adapter.
    """

    errors: list[str] = []
    if payload.get("schema") != "taoryx.vehicle-integration-intake/v1alpha1":
        errors.append("schema must be taoryx.vehicle-integration-intake/v1alpha1")

    kind = payload.get("kind")
    if kind not in {"existing_family", "new_topology"}:
        errors.append("kind must be existing_family or new_topology")

    raw_tiers = payload.get("tiers")
    if not isinstance(raw_tiers, list):
        errors.append("tiers must be a list")
    else:
        supplied_tiers: list[str] = []
        for index, item in enumerate(raw_tiers):
            if not isinstance(item, Mapping):
                errors.append(f"tiers[{index}] must be a mapping")
                continue
            tier = item.get("tier")
            if not isinstance(tier, str):
                errors.append(f"tiers[{index}].tier must be a string")
                continue
            supplied_tiers.append(tier)
            if tier not in CANONICAL_FIDELITY_TIERS:
                errors.append(f"tiers[{index}].tier {tier!r} is not canonical")
                continue
            expected_realization = control_realization_for(tier)
            if item.get("control_realization") != expected_realization:
                errors.append(f"tiers[{index}] control realization must be {expected_realization!r} for {tier!r}")
        if tuple(supplied_tiers) != CANONICAL_FIDELITY_TIERS:
            errors.append("tiers must contain each canonical fidelity tier once in canonical order")

    interface = payload.get("interface_contract_required")
    if not isinstance(interface, Mapping):
        errors.append("interface_contract_required must be a mapping")
    else:
        if interface.get("status") != "required_before_execution_binding":
            errors.append("interface_contract_required.status must block execution binding")
        for key in ("parameter_contract", "control_contract", "status_observation_resource_contract"):
            if not isinstance(interface.get(key), Mapping):
                errors.append(f"interface_contract_required.{key} must be a mapping")
        execution_mode_contract = interface.get("execution_mode_contract")
        if not isinstance(execution_mode_contract, Mapping):
            errors.append("interface_contract_required.execution_mode_contract must be a mapping")
        else:
            if execution_mode_contract.get("status") != "required_before_execution_binding":
                errors.append("interface_contract_required.execution_mode_contract.status must block execution binding")
            allowed_values = execution_mode_contract.get("allowed_values")
            if not isinstance(allowed_values, list) or tuple(allowed_values) != get_args(ExecutionMode):
                errors.append("interface_contract_required.execution_mode_contract.allowed_values must match the canonical execution-mode vocabulary")
            required_per_endpoint = execution_mode_contract.get("required_per_endpoint")
            if not isinstance(required_per_endpoint, list) or "execution_mode" not in required_per_endpoint:
                errors.append("interface_contract_required.execution_mode_contract must require execution_mode per endpoint")
        execution = interface.get("execution_and_evaluation_contract")
        if not isinstance(execution, list):
            errors.append("interface_contract_required.execution_and_evaluation_contract must be a list")
        else:
            required_execution_contracts = {
                "exact_batch_and_episode_factory_or_declared_gap",
                "execution_mode_and_batch_episode_disposition",
                "batch_action_trace_disposition",
                "committed_truth_boundary_and_sensor_cadence_policy",
                "mission_objective_and_terminal_semantics",
                "normalized_result_and_provenance_artifacts",
                "replay_or_parity_witness_plan_when_both_modes_are_advertised",
            }
            missing_execution_contracts = sorted(required_execution_contracts.difference(item for item in execution if isinstance(item, str)))
            if missing_execution_contracts:
                errors.append(
                    "interface_contract_required.execution_and_evaluation_contract lacks required entries: "
                    + ", ".join(missing_execution_contracts)
                )

    mission_binding_contract = payload.get("mission_binding_contract_required")
    if not isinstance(mission_binding_contract, Mapping):
        errors.append("mission_binding_contract_required must be a mapping")
    else:
        if mission_binding_contract.get("status") != "required_before_mission_preflight":
            errors.append("mission_binding_contract_required.status must block mission preflight")
        supported_kinds = mission_binding_contract.get("supported_kinds")
        if not isinstance(supported_kinds, list) or tuple(supported_kinds) != ("legacy_racetrack", "semantic_composition"):
            errors.append("mission_binding_contract_required.supported_kinds must retain canonical mission binding kinds")
        semantic_contract = mission_binding_contract.get("semantic_composition")
        if not isinstance(semantic_contract, Mapping):
            errors.append("mission_binding_contract_required.semantic_composition must be a mapping")
        else:
            fields = semantic_contract.get("required_fields")
            if not isinstance(fields, list) or not any("composition_family_id" in str(item) for item in fields):
                errors.append("mission_binding_contract_required.semantic_composition must require composition_family_id")
            realization_fields = semantic_contract.get("required_per_realization")
            if not isinstance(realization_fields, list) or not any("expected_preflight_status" in str(item) for item in realization_fields):
                errors.append("mission_binding_contract_required.semantic_composition must require expected_preflight_status per realization")

    raw_witnesses = payload.get("synthetic_conformance_witness_plan")
    if not isinstance(raw_witnesses, list):
        errors.append("synthetic_conformance_witness_plan must be a list")
    else:
        witness_tiers: list[str] = []
        for index, item in enumerate(raw_witnesses):
            if not isinstance(item, Mapping):
                errors.append(f"synthetic_conformance_witness_plan[{index}] must be a mapping")
                continue
            tier = item.get("tier")
            if not isinstance(tier, str):
                errors.append(f"synthetic_conformance_witness_plan[{index}].tier must be a string")
                continue
            witness_tiers.append(tier)
            if tier not in CANONICAL_FIDELITY_TIERS:
                errors.append(f"synthetic_conformance_witness_plan[{index}].tier {tier!r} is not canonical")
                continue
            if item.get("control_realization") != control_realization_for(tier):
                errors.append(f"synthetic_conformance_witness_plan[{index}] has a mismatched control realization")
            required_witnesses = item.get("required_witnesses")
            if not isinstance(required_witnesses, list) or not required_witnesses:
                errors.append(f"synthetic_conformance_witness_plan[{index}] requires at least one witness")
        if tuple(witness_tiers) != CANONICAL_FIDELITY_TIERS:
            errors.append("synthetic_conformance_witness_plan must cover canonical tiers in canonical order")

    if kind == "existing_family":
        registry = payload.get("registry_entry_template")
        if not isinstance(registry, Mapping):
            errors.append("existing-family intake requires registry_entry_template")
        else:
            registry_tiers = registry.get("tiers")
            if not isinstance(registry_tiers, Mapping) or tuple(registry_tiers) != CANONICAL_FIDELITY_TIERS:
                errors.append("registry_entry_template.tiers must cover canonical tiers in canonical order")
    elif kind == "new_topology":
        if not isinstance(payload.get("source_manifest_template"), Mapping):
            errors.append("new-topology intake requires source_manifest_template")
        if not isinstance(payload.get("new_family_decision_record"), Mapping):
            errors.append("new-topology intake requires new_family_decision_record")

    return tuple(errors)
    ####


def _with_intake_validation(payload: dict[str, object]) -> dict[str, object]:
    """Attach a checked generation result, failing closed on an invalid template."""

    errors = validate_integration_intake_payload(payload)
    if errors:
        raise ValueError(f"generated intake payload is invalid: {'; '.join(errors)}")
    payload["intake_validation"] = {
        "status": "pass",
        "canonical_fidelity_tiers": list(CANONICAL_FIDELITY_TIERS),
        "errors": [],
        "claim_boundary": ("Structural intake conformance only. It does not validate source data, adapter behavior, or promotion evidence."),
    }
    return payload
    ####


__all__ = [
    "ExistingFamilyIntakeBlueprint",
    "ExistingFamilyIntakeRequest",
    "IntakeKind",
    "IntakeTierBlueprint",
    "NewTopologyIntakeRequest",
    "NewTopologyIntakeScaffold",
    "StrategySelectionError",
    "build_existing_family_intake_blueprint",
    "build_new_topology_intake_scaffold",
    "compatible_strategies",
    "select_existing_family_strategy",
    "validate_integration_intake_payload",
]
