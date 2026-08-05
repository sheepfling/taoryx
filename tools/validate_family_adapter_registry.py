#!/usr/bin/env python3
"""Run the common adapter conformance harness against executable witnesses.

The horizontal YAML registry names every supported family.  This executable
registry intentionally contains only the adapters that can be constructed by
the current runtime slice; its X8, B747, Hummingbird, and F-16 source
witnesses use the same factories as the Product 3 runtime registry.  The
remaining registrations are marked ``planned`` so the report distinguishes
missing integration work from a failed witness.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Callable
from functools import lru_cache
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.family_adapter import (
    AdapterChannel,
    FamilyAdapterDescriptor,
    StandardFamilyAdapter,
    descriptor_from_control_plant,
)
from taoryx.family_adapter_probes import AdapterProbeCase
from taoryx.family_adapter_registry import (
    AdapterRegistryReport,
    FamilyAdapterRegistration,
    FamilyAdapterRegistry,
)
from taoryx.fidelity_contracts import FidelityTier
from taoryx.hl20_adapter import build_hl20_source_adapter
from taoryx.horizontal_fidelity import load_horizontal_registry, validate_horizontal_fidelity
from taoryx.nesc_adapter import build_nesc_replay_adapter
from taoryx.source_f16 import build_f16_source_physical_plant
from taoryx.source_table_fixed_wing import (
    build_b747_condition3_source_table_plant,
    build_x8_source_table_plant,
)
from taoryx.source_table_multirotor import build_hummingbird_individual_rotor_source_table_plant
from taoryx.trajectory import (
    A320OpenAPControlPlant,
    A320OpenAPModel,
    A320Pseudo6DOFControlPlant,
    A320Pseudo6DOFModel,
    F16ReferencePhysicalPlant,
)
from taoryx.trajectory.f16_reduced_adapter import F16PointMassControlPlant, F16Pseudo6DOFControlPlant
from taoryx.trajectory.f16_reductions import F16AttitudeResponsePseudo6DOFModel, F16PointMass3DOFModel
from taoryx.trajectory.hummingbird_adapter import build_hummingbird_reduced_control_plant
from taoryx.trajectory.reduced_control_plant import ReducedOrderControlPlant
from taoryx.x15_adapter import (
    build_x15_source_direct_wrench_adapter,
    build_x15_source_direct_wrench_plant,
)

OUTPUT = ROOT / "verification/alpha3_horizontal_fidelity/adapter_registry.json"
_SURFACE: FidelityTier = "rigid_body_6dof_surface_allocated"
_DIRECT: FidelityTier = "rigid_body_6dof_direct_wrench"
_PSEUDO: FidelityTier = "pseudo_6dof"


def _controlled_factory(
    builder: Callable[[], Any],
    *,
    family_id: str,
    adapter_id: str,
    physical_family: str,
) -> Callable[[FidelityTier], StandardFamilyAdapter]:
    """Wrap one existing table-backed plant without duplicating its schemas."""

    @lru_cache(maxsize=1)
    def plant() -> Any:
        """Construct the immutable witness plant once for every tier matrix."""

        return builder()
        ####

    @lru_cache(maxsize=None)
    def build(tier: FidelityTier) -> StandardFamilyAdapter:
        resolved_plant = plant()
        limits = getattr(resolved_plant, "effector_limits", None) or getattr(resolved_plant, "effectors", None)
        if limits is None:
            raise ValueError(f"{family_id}: witness plant has no effector limits")
        units = {name: limits[name].unit for name in resolved_plant.control_names}
        descriptor = descriptor_from_control_plant(
            resolved_plant,
            family_id=family_id,
            adapter_id=adapter_id,
            physical_family=physical_family,
            tier=tier,
            control_units=units,
            evidence_status="development",
            omitted_physics=("family-specific mission and resource providers",),
        )
        return StandardFamilyAdapter.from_control_plant(descriptor, resolved_plant)

    return build
    ####


def _reduced_factory(
    builder: Callable[[FidelityTier], ReducedOrderControlPlant],
    *,
    family_id: str,
    adapter_id: str,
    physical_family: str,
    omitted_physics: tuple[str, ...],
) -> Callable[[FidelityTier], StandardFamilyAdapter]:
    """Bind a declared lower-tier force or response law to common tooling."""

    @lru_cache(maxsize=None)
    def build(tier: FidelityTier) -> StandardFamilyAdapter:
        plant = builder(tier)
        descriptor = descriptor_from_control_plant(
            plant,
            family_id=family_id,
            adapter_id=adapter_id,
            physical_family=physical_family,
            tier=tier,
            state_units=plant.state_units,
            control_units=plant.control_units,
            evidence_status="development",
            omitted_physics=omitted_physics,
        )
        return StandardFamilyAdapter.from_control_plant(descriptor, plant)

    return build
    ####


def _source_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Use the plant's declared source operating point for generic probes."""

    plant = adapter.plant
    if plant is None:
        raise ValueError(f"{adapter.describe().family_id}: witness plant has no source operating point")
    if hasattr(plant, "source_local_state") and hasattr(plant, "source_effectors"):
        state = dict(getattr(plant, "source_local_state"))
        effectors = dict(getattr(plant, "source_effectors"))
        environment: dict[str, float | str] = {}
    elif hasattr(plant, "trim_result"):
        trim = getattr(plant, "trim_result")
        state = dict(trim.state)
        effectors = dict(trim.controls)
        environment = {
            "altitude_m": float(getattr(plant, "altitude_m", 0.0)),
            "trim_pitch_rad": float(getattr(plant, "trim_pitch_rad", 0.0)),
        }
    else:
        raise ValueError(f"{adapter.describe().family_id}: witness plant has no source operating point")
    return AdapterProbeCase(
        state=state,
        effectors=effectors,
        environment=environment,
        trim_target=state,
        trim_initial_guess=effectors,
        previous_effectors=effectors,
    )
    ####


def _trim_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Build a common lower-tier probe directly from its declared trim state."""

    trim = adapter.trim({}, {})
    return AdapterProbeCase(
        state=dict(trim.state),
        effectors=dict(trim.controls),
        trim_target=dict(trim.state),
        trim_initial_guess=dict(trim.controls),
        previous_effectors=dict(trim.controls),
    )
    ####


@lru_cache(maxsize=None)
def _build_hummingbird_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Select reduced or native Hummingbird evidence without tier fallback."""

    if tier in {"point_mass_3dof", "pseudo_6dof"}:
        return _reduced_factory(
            build_hummingbird_reduced_control_plant,
            family_id="hummingbird",
            adapter_id="taoryx.multirotor.native_quad_x.v1",
            physical_family="multirotor",
            omitted_physics=("individual rotor allocation", "rotor inflow", "physical motor dynamics"),
        )(tier)
    return _controlled_factory(
        build_hummingbird_individual_rotor_source_table_plant,
        family_id="hummingbird",
        adapter_id="taoryx.multirotor.native_quad_x.v1",
        physical_family="multirotor",
    )(tier)
    ####


@lru_cache(maxsize=1)
def _build_f16_plant() -> F16ReferencePhysicalPlant:
    """Build the runtime-owned source-backed F-16 plant."""

    return build_f16_source_physical_plant()
    ####


def _build_f16_reduced_plant(tier: FidelityTier) -> ReducedOrderControlPlant:
    """Project the F-16 source trim into an explicit reduced-model adapter."""

    physical = _build_f16_plant()
    point = F16PointMass3DOFModel(
        physical.source,
        physical.trim_result,
        physical.trim_pitch_rad,
        physical.altitude_m,
    )
    if tier == "point_mass_3dof":
        return F16PointMassControlPlant(point)
    if tier == "pseudo_6dof":
        source_linearization = physical.linearize(physical.trim_result, {})
        pseudo = F16AttitudeResponsePseudo6DOFModel(
            physical.source,
            physical.trim_result,
            source_linearization,
            physical.trim_pitch_rad,
            physical.altitude_m,
        )
        return F16Pseudo6DOFControlPlant(pseudo)
    raise ValueError(f"F-16 has no reduced control plant for {tier}")
    ####


@lru_cache(maxsize=None)
def _build_f16_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Select reduced or physical F-16 evidence without tier fallback."""

    if tier in {"point_mass_3dof", "pseudo_6dof"}:
        return _reduced_factory(
            _build_f16_reduced_plant,
            family_id="f16_s119",
            adapter_id="taoryx.fixed_wing.daveml.v1",
            physical_family="powered_fixed_wing",
            omitted_physics=("physical attitude moments", "actuator dynamics", "surface allocation"),
        )(tier)
    return _controlled_factory(
        _build_f16_plant,
        family_id="f16_s119",
        adapter_id="taoryx.fixed_wing.daveml.v1",
        physical_family="powered_fixed_wing",
    )(tier)
    ####


def _reduced_or_source_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Use a trim probe for reduced tiers and source fixtures for native tiers."""

    if adapter.describe().tier in {"point_mass_3dof", "pseudo_6dof"}:
        return _trim_probe(adapter)
    return _source_probe(adapter)
    ####


def _build_a320_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Build the executable A320 tier that has a real reduced-order product."""

    plant: A320OpenAPControlPlant | A320Pseudo6DOFControlPlant
    if tier == "point_mass_3dof":
        plant = A320OpenAPControlPlant(A320OpenAPModel.from_repository(ROOT))
        state_units = {"altitude_m": "m", "mach": "1", "mass_kg": "kg", "range_m": "m"}
        control_units = {"throttle_ratio": "1", "flight_path_angle_rad": "rad"}
    elif tier == "pseudo_6dof":
        plant = A320Pseudo6DOFControlPlant(A320Pseudo6DOFModel.from_repository(ROOT))
        state_units = {
            "altitude_m": "m",
            "mach": "1",
            "mass_kg": "kg",
            "range_m": "m",
            "alpha_rad": "rad",
            "beta_rad": "rad",
            "roll_rate_rad_s": "rad/s",
            "pitch_rate_rad_s": "rad/s",
            "yaw_rate_rad_s": "rad/s",
            "bank_angle_rad": "rad",
        }
        control_units = {"throttle_ratio": "1", "flight_path_angle_rad": "rad", "aileron_rad": "rad", "elevator_rad": "rad", "rudder_rad": "rad"}
    else:
        raise ValueError(f"a320_openap_3dof has no executable {tier} adapter yet")
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="a320_openap_3dof",
        adapter_id="taoryx.fixed_wing.openap.v1",
        physical_family="powered_fixed_wing",
        tier=tier,
        state_units=state_units,
        control_units=control_units,
        evidence_status="development",
        omitted_physics=("physical surface allocation", "manufacturer-authoritative flight dynamics"),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _a320_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Build a source operating point for the A320 reduced products."""

    plant = adapter.plant
    if isinstance(plant, A320OpenAPControlPlant):
        point = plant.operating_point
        baseline = plant.model.evaluate(point)
        state = {
            "altitude_m": point.altitude_m,
            "mach": point.mach,
            "mass_kg": point.mass_kg,
            "range_m": 0.0,
        }
        effectors = {"throttle_ratio": baseline.required_throttle_ratio, "flight_path_angle_rad": 0.0}
    elif isinstance(plant, A320Pseudo6DOFControlPlant):
        point = plant.operating_point
        solved = plant.model.trim_pseudo6dof(point)
        state = {
            "altitude_m": point.altitude_m,
            "mach": point.mach,
            "mass_kg": point.mass_kg,
            "range_m": 0.0,
            "alpha_rad": float(solved.state.get("alpha_rad", 0.0)),
            "beta_rad": 0.0,
            "roll_rate_rad_s": 0.0,
            "pitch_rate_rad_s": 0.0,
            "yaw_rate_rad_s": 0.0,
            "bank_angle_rad": 0.0,
        }
        effectors = {name: float(value) for name, value in solved.controls.items()}
    else:
        raise ValueError("unexpected A320 plant type")
    return AdapterProbeCase(
        state=state,
        effectors=effectors,
        environment={"thrust_mode": point.thrust_mode},
        trim_target=state,
        trim_initial_guess=effectors,
        previous_effectors=effectors,
    )
    ####


def _hl20_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Build the declared local Mach-1, alpha-5 source-surface witness."""

    speed = 340.294
    alpha_rad = 5.0 * 3.141592653589793 / 180.0
    state = {
        "u_m_s": speed * math.cos(alpha_rad),
        "v_m_s": 0.0,
        "w_m_s": -speed * math.sin(alpha_rad),
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
        "altitude_m": 0.0,
    }
    effectors = {name: 0.0 for name in adapter.control_names}
    return AdapterProbeCase(
        state=state,
        effectors=effectors,
        trim_target=state,
        trim_initial_guess=effectors,
        previous_effectors=effectors,
    )
    ####


def _x15_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Build a local release/glide operating point for the direct bridge."""

    plant = build_x15_source_direct_wrench_plant()
    effectors = {name: 0.0 for name in adapter.control_names}
    return AdapterProbeCase(
        state=dict(plant.reference_state),
        effectors=effectors,
        trim_target=dict(plant.reference_state),
        trim_initial_guess=effectors,
        previous_effectors=effectors,
    )
    ####


def _nesc_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Build a replay request for the pinned NESC reduction artifact."""

    del adapter
    return AdapterProbeCase(
        state={},
        effectors={},
        replay_request={"source_artifact": str(ROOT / "verification/daveml_nesc_reduction_qualification.json")},
    )
    ####


def _passive_tumbling_factory(tier: FidelityTier) -> StandardFamilyAdapter:
    """Represent the tumbling topology without fabricating controller inputs."""

    descriptor = FamilyAdapterDescriptor(
        family_id="tumbling_body",
        adapter_id="taoryx.passive_body.rigid_aero.v1",
        physical_family="passive_ballistic_tumbling_body",
        tier=tier,
        state_channels=(
            AdapterChannel("position_ned_m", "m", "state", frame="NED"),
            AdapterChannel("velocity_body_m_s", "m/s", "state", frame="body"),
            AdapterChannel("attitude_quaternion", "unitless", "state", frame="body-to-NED"),
            AdapterChannel("body_rate_rad_s", "rad/s", "state", frame="body"),
        ),
        control_realization_override="uncontrolled",
        evidence_status="development",
        omitted_physics=("controller", "allocator", "actuator dynamics"),
    )
    return StandardFamilyAdapter.passive(descriptor)
    ####


def build_registry() -> FamilyAdapterRegistry:
    """Return the executable witness registry for the current integration slice."""

    registrations = (
        FamilyAdapterRegistration(
            "skywalker_x8",
            "taoryx.fixed_wing.source_table.v1",
            "available",
            _controlled_factory(
                build_x8_source_table_plant,
                family_id="skywalker_x8",
                adapter_id="taoryx.fixed_wing.source_table.v1",
                physical_family="powered_fixed_wing",
            ),
            probe_factory=_source_probe,
            supported_tiers=(_DIRECT, _SURFACE),
            note="source-table local plant supports direct and surface witness tiers",
        ),
        FamilyAdapterRegistration(
            "b747",
            "taoryx.fixed_wing.source_table.v1",
            "available",
            _controlled_factory(
                build_b747_condition3_source_table_plant,
                family_id="b747",
                adapter_id="taoryx.fixed_wing.source_table.v1",
                physical_family="powered_fixed_wing",
            ),
            probe_factory=_source_probe,
            supported_tiers=(_DIRECT, _SURFACE),
            note="condition-3 source-table local plant supports direct and surface witness tiers",
        ),
        FamilyAdapterRegistration(
            "hummingbird",
            "taoryx.multirotor.native_quad_x.v1",
            "available",
            _build_hummingbird_adapter,
            probe_factory=_reduced_or_source_probe,
            supported_tiers=("point_mass_3dof", _PSEUDO, _DIRECT, _SURFACE),
            note="aggregate lower tiers and individual-rotor direct/surface witnesses are explicit and separately probed",
        ),
        FamilyAdapterRegistration(
            "tumbling_body",
            "taoryx.passive_body.rigid_aero.v1",
            "available",
            _passive_tumbling_factory,
            supported_tiers=(_PSEUDO,),
            note="passive topology witness; no controlled tier is applicable",
        ),
        FamilyAdapterRegistration(
            "a320_openap_3dof",
            "taoryx.fixed_wing.openap.v1",
            "available",
            _build_a320_adapter,
            probe_factory=_a320_probe,
            supported_tiers=("point_mass_3dof", "pseudo_6dof"),
            note="OpenAP point-mass and named pseudo-6DOF products use the common façade; direct and surface tiers remain planned",
        ),
        FamilyAdapterRegistration(
            "f16_s119",
            "taoryx.fixed_wing.daveml.v1",
            "available",
            _build_f16_adapter,
            probe_factory=_reduced_or_source_probe,
            supported_tiers=("point_mass_3dof", _PSEUDO, _DIRECT, _SURFACE),
            note="source-force reduced tiers and source-backed direct/surface witnesses are explicit and separately probed",
        ),
        FamilyAdapterRegistration(
            "x15",
            "taoryx.high_energy.fixed_wing.v1",
            "available",
            build_x15_source_direct_wrench_adapter,
            probe_factory=_x15_probe,
            supported_tiers=(_DIRECT,),
            note="source rigid-body local release/glide loads are executable through the explicit direct-wrench bridge; physical effectors and regime scheduling remain planned",
        ),
        FamilyAdapterRegistration(
            "hl20_mod_k",
            "taoryx.lifting_body.daveml.v1",
            "available",
            build_hl20_source_adapter,
            probe_factory=_hl20_probe,
            supported_tiers=(_DIRECT, _SURFACE),
            note="source-backed local direct-wrench and seven-surface allocation witnesses are executable; trim and closed-loop flight remain planned",
        ),
        FamilyAdapterRegistration(
            "reference_nesc_two_stage_rocket",
            "taoryx.rocket.variable_mass_nesc.v1",
            "available",
            build_nesc_replay_adapter,
            probe_factory=_nesc_probe,
            supported_tiers=("point_mass_3dof", "pseudo_6dof"),
            note="source translation and scheduled response are executable through replay; participating derivative and physical gimbal tiers remain planned",
        ),
    )
    return FamilyAdapterRegistry(registrations)
    ####


def _manifest_alignment(registry: FamilyAdapterRegistry) -> dict[str, object]:
    manifest_ids = {item.family_id for item in load_horizontal_registry().families}
    registered_ids = {item.family_id for item in registry.registrations}
    missing = sorted(manifest_ids - registered_ids)
    unexpected = sorted(registered_ids - manifest_ids)
    return {
        "status": "pass" if not missing and not unexpected else "blocked",
        "missing_manifest_families": missing,
        "unexpected_registered_families": unexpected,
    }
    ####


def _promotion_matrix(tier_matrix: AdapterRegistryReport) -> dict[str, object]:
    """Evaluate declared tier gates against the common adapter probes."""

    checks = {(item.family_id, item.tier): item for item in tier_matrix.checks}
    entries: list[dict[str, object]] = []
    qualified_failures: list[str] = []
    declared_counts: dict[str, int] = {}
    validation_counts: dict[str, int] = {}
    for family in load_horizontal_registry().families:
        for tier, binding in family.tiers.items():
            check = checks.get((family.family_id, tier))
            operation_status: dict[str, str] = {}
            if check is not None and check.probe is not None:
                operation_status = {item.operation: item.status for item in check.probe.operations}
            missing_operations = [
                operation
                for operation in binding.required_operations
                if operation_status.get(operation) != "pass"
            ]
            if binding.promotion_status == "not_applicable":
                validation_status = "not_applicable"
            elif binding.promotion_status == "planned":
                validation_status = "planned"
            elif check is None:
                validation_status = "not_checked"
            elif check.status != "pass" or missing_operations:
                validation_status = "blocked"
            else:
                validation_status = "pass"
            blockers = list(binding.blockers)
            blockers.extend(f"operation_{operation}_not_passed" for operation in missing_operations)
            if binding.promotion_status == "qualified" and validation_status != "pass":
                qualified_failures.append(f"{family.family_id}:{tier}")
            declared_counts[binding.promotion_status] = declared_counts.get(binding.promotion_status, 0) + 1
            validation_counts[validation_status] = validation_counts.get(validation_status, 0) + 1
            entries.append(
                {
                    "family_id": family.family_id,
                    "tier": tier,
                    "profile_id": binding.profile_id,
                    "declared_status": binding.promotion_status,
                    "validation_status": validation_status,
                    "required_operations": list(binding.required_operations),
                    "operation_status": operation_status,
                    "blockers": blockers,
                }
            )
    return {
        "schema": "taoryx.family-tier-promotion/v1alpha1",
        "status": "blocked" if qualified_failures else "development",
        "entries": entries,
        "declared_status_counts": declared_counts,
        "validation_status_counts": validation_counts,
        "qualified_failures": qualified_failures,
    }
    ####


def build_report() -> dict[str, object]:
    """Build the reproducible registry/conformance artifact."""

    registry = build_registry()
    tiers: dict[str, FidelityTier] = {
        "skywalker_x8": _SURFACE,
        "b747": _SURFACE,
        "hummingbird": _SURFACE,
        "f16_s119": _SURFACE,
        "tumbling_body": _PSEUDO,
        "a320_openap_3dof": _PSEUDO,
        "hl20_mod_k": _SURFACE,
        "x15": _DIRECT,
        "reference_nesc_two_stage_rocket": _PSEUDO,
    }
    report: AdapterRegistryReport = registry.check_all(tiers)
    tier_matrix = registry.check_matrix()
    promotion = _promotion_matrix(tier_matrix)
    horizontal_lowering = validate_horizontal_fidelity(
        adapter_operation_status=tier_matrix.operation_status_by_family_tier()
    )
    alignment = _manifest_alignment(registry)
    status = (
        "blocked"
        if (
            alignment["status"] == "blocked"
            or report.status == "blocked"
            or tier_matrix.status == "blocked"
            or promotion["status"] == "blocked"
            or horizontal_lowering.errors
        )
        else report.status
    )
    if status == "pass":
        status = "development"
    executable_witness_count = sum(item.status == "available" for item in registry.registrations)
    planned_family_count = sum(item.status == "planned" for item in registry.registrations)
    return {
        "schema": "taoryx.family-adapter-registry/v1alpha1",
        "status": status,
        "manifest": "verification/horizontal_fidelity_registry.yaml",
        "alignment": alignment,
        "executable_witness_count": executable_witness_count,
        "planned_family_count": planned_family_count,
        "registry": report.as_dict(),
        "tier_matrix": tier_matrix.as_dict(),
        "horizontal_lowering": horizontal_lowering.as_dict(),
        "promotion_matrix": promotion,
        "declared_tier_check_count": len(tier_matrix.checks),
        "claim_boundary": "This is adapter-contract conformance evidence; it is not a vehicle-family qualification or a claim that every advertised tier is implemented.",
        "reproduction": "PYTHONPATH=src python3 tools/validate_family_adapter_registry.py --check",
    }
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--check", action="store_true", help="fail on blocked alignment or conformance")
    args = parser.parse_args()
    payload = build_report()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"status": payload["status"], "executable_witness_count": payload["executable_witness_count"], "planned_family_count": payload["planned_family_count"]}))
    return 1 if args.check and payload["status"] == "blocked" else 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
