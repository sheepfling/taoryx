"""Contract-only future-family interface probes for Alpha 2."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from taoryx.trajectory import (
    AllocationSchema,
    CapabilitySchema,
    CaseIntent,
    ComponentSlot,
    ControlSchema,
    FamilyCatalog,
    FamilyPackage,
    ModeTransitionSchema,
    ObservationSchema,
    ResolutionError,
    ResourceSchema,
    resolve_case,
)
from tools.generate_alpha2_interface_report import build_report

ROOT = Path(__file__).resolve().parents[2]
MATRIX = ROOT / "verification/alpha2_interface_stress_matrix.yaml"
REPORT = ROOT / "verification/generated/alpha2_interface_compatibility_report.json"


def _package(probe: dict[str, object]) -> FamilyPackage:
    capabilities = dict(probe["capabilities"])
    capabilities["fidelities"] = tuple(probe["fidelities"])
    capabilities["control_intents"] = tuple(capabilities["control_intents"])
    capabilities["observation_kinds"] = tuple(capabilities["observation_kinds"])
    controls = tuple(
        ControlSchema(
            id=name,
            command_modes=("absolute", "rate") if name.endswith("_rate") else ("absolute",),
            rate_unit="unit/s" if name.endswith("_rate") else None,
            frame="body" if name not in {"throttle", "power_request"} else None,
            semantic_level="effector",
            achieved_observation_id=f"{name}_achieved",
            allocation_id=str(probe["allocations"][0]),
            evidence_grade="mixed",
        )
        for name in probe["controls"]
    )
    observations = tuple(
        ObservationSchema(
            id=name,
            kind="resource" if name.endswith(("_remaining", "_energy", "_power")) else "state",
            frame="body" if name in {"quaternion", "orbital_frame"} else None,
            semantic_level="resource" if name.endswith(("_remaining", "_energy", "_power")) else "state",
            evidence_grade="mixed",
            neutral_value=0.0,
        )
        for name in probe["observations"]
    )
    slots = tuple(
        ComponentSlot(id=name, component_kind=name.replace("_", "-"), compatible_fidelities=tuple(probe["fidelities"]), provenance="stress-fixture")
        for name in probe["component_slots"]
    )
    resources = tuple(
        ResourceSchema(
            id=name,
            resource_kind=name.removesuffix("_mass").removesuffix("_energy"),
            canonical_unit="kg" if name.endswith("_mass") else "J",
            default=1.0,
            reserve=0.1,
            observation_id=f"{name}_remaining",
            evidence_grade="synthetic",
            provenance="stress-fixture",
        )
        for name in probe["resources"]
    )
    allocations = tuple(
        AllocationSchema(
            id=name,
            requested_channels=tuple(probe["controls"]),
            allocated_channels=tuple(probe["controls"]),
            achieved_observations=tuple(f"{control}_achieved" for control in probe["controls"]),
            supports_rate_commands=True,
            provenance="stress-fixture",
        )
        for name in probe["allocations"]
    )
    transitions = tuple(
        ModeTransitionSchema(
            id=name,
            from_mode="mode_a",
            to_mode="mode_b",
            entry_guards=("valid_state",),
            exit_guards=("target_reached",),
            abort_to="mode_a",
            hysteresis_s=0.1,
            schedules={"authority": "smooth"},
            controller_handoff={"reset_integrator": True},
            provenance="stress-fixture",
        )
        for name in probe["mode_transitions"]
    )
    return FamilyPackage(
        family_id=str(probe["id"]),
        version="1.0.0",
        display_name=str(probe["id"]),
        fidelities=tuple(probe["fidelities"]),
        parameters=(),
        variants={"baseline": {}},
        loadouts={"baseline": {}},
        missions={"baseline": {}},
        segment_plans={"baseline": {}},
        controls=controls,
        observations=observations,
        capabilities=CapabilitySchema.model_validate(capabilities),
        component_slots=slots,
        resources=resources,
        allocations=allocations,
        mode_transitions=transitions,
        evidence_grade="mixed",
        uncertainty=str(probe["evidence"]["uncertainty"]),
        provenance="contract-only stress fixture",
    )
    ####


def _probes() -> list[dict[str, object]]:
    payload = yaml.safe_load(MATRIX.read_text(encoding="utf-8"))
    return list(payload["probes"])
    ####


def test_future_family_matrix_covers_typed_contract_surface() -> None:
    probes = _probes()
    assert len(probes) == 6
    required = {"capabilities", "component_slots", "controls", "observations", "resources", "allocations", "mode_transitions", "evidence"}
    for probe in probes:
        assert required <= set(probe)
        assert set(probe["fidelities"]) == {"point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"}
        assert probe["capabilities"]["supports_checkpoint"] is True
        assert probe["resources"]
        assert probe["mode_transitions"]
    ####


def test_generated_interface_report_is_current_and_claim_bounded() -> None:
    matrix = yaml.safe_load(MATRIX.read_text(encoding="utf-8"))
    report = yaml.safe_load(REPORT.read_text(encoding="utf-8"))
    assert report == build_report(matrix)
    assert report["status"] == "pass"
    assert report["claim_boundary"] == "contract coverage only; no provider qualification"
    assert all(item["provider_status"] == "interface_only" for item in report["findings"])
    ####


@pytest.mark.parametrize("probe", _probes(), ids=lambda probe: str(probe["id"]))
def test_future_family_probe_resolves_without_bespoke_runner(probe: dict[str, object]) -> None:
    package = _package(probe)
    catalog = FamilyCatalog(schema_version=1, families=(package,))
    intent = CaseIntent(
        case_id=f"{package.family_id}-interface-probe",
        family=package.family_id,
        fidelity="rigid_body_6dof",
        requested_controls=tuple(control.id for control in package.controls),
        requested_observations=tuple(observation.id for observation in package.observations),
    )
    resolved = resolve_case(intent, catalog)
    assert resolved.identity_sha256 == resolved.recompute_identity()
    assert resolved.capabilities.supports_resources
    assert resolved.component_slots
    assert resolved.resources
    assert resolved.allocations
    assert resolved.mode_transitions
    assert resolved.evidence_grade == "mixed"
    ####


def test_unsupported_fidelity_and_unavailable_channel_fail_during_resolution() -> None:
    package = _package(_probes()[0]).model_copy(update={"fidelities": ("point_mass_3dof",)})
    catalog = FamilyCatalog(schema_version=1, families=(package,))
    with pytest.raises(ResolutionError, match="unsupported-fidelity"):
        resolve_case(CaseIntent(case_id="bad-fidelity", family=package.family_id, fidelity="rigid_body_6dof"), catalog)

    unavailable = package.controls[0].model_copy(update={"availability": "unavailable"})
    package = package.model_copy(update={"controls": (unavailable, *package.controls[1:])})
    with pytest.raises(ResolutionError, match="unsupported-control"):
        resolve_case(
            CaseIntent(case_id="bad-control", family=package.family_id, fidelity="point_mass_3dof", requested_controls=(unavailable.id,)),
            FamilyCatalog(schema_version=1, families=(package,)),
        )
    ####
