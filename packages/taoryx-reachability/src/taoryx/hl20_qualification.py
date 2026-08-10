"""Machine-readable HL-20 G0-G6 qualification gates.

The gates validate emitted reachability artifacts, not private simulator
objects. This keeps the qualification report useful to downstream analysis
and catches broken serialization, provenance, or deployment lineage.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml
from taoryx_reachability.resources import reachability_resource_root

HL20_GATE_IDS = ("HL20-G0", "HL20-G1", "HL20-G2", "HL20-G3", "HL20-G4", "HL20-G5", "HL20-G6")
DEFAULT_QUALITY_GATES = reachability_resource_root() / "examples/showcases/hl20_california_to_hawaii/quality_gates.yaml"
DEFAULT_ARTIFACT_DIR = Path("artifacts/showcases/hl20_california_to_hawaii/low_fidelity")
_FIDELITIES = (
    "point_mass_3dof",
    "pseudo_6dof",
    "rigid_body_6dof",
    "rigid_body_6dof_surface_allocated",
)
_ARTIFACT_SCHEMA = "trajectory.reachability-envelope/v1alpha1"
_PHASE_ORDER = ("boost", "coast", "glide")


@dataclass(frozen=True, slots=True)
class HL20GateResult:
    """One ordered qualification gate and its compact evidence."""

    gate_id: str
    number: int
    name: str
    status: str
    message: str
    evidence: Mapping[str, object]

    def as_dict(self) -> dict[str, object]:
        return {
            "id": self.gate_id,
            "number": self.number,
            "name": self.name,
            "status": self.status,
            "message": self.message,
            "evidence": dict(self.evidence),
        }


@dataclass(frozen=True, slots=True)
class HL20QualificationReport:
    """Serializable verdict for the HL-20 qualification ladder."""

    schema: str
    scenario_id: str
    vehicle_id: str
    artifact_dir: str
    verdict: str
    gates: tuple[HL20GateResult, ...]
    fidelities: tuple[str, ...]
    contract_path: str

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "scenario_id": self.scenario_id,
            "vehicle_id": self.vehicle_id,
            "artifact_dir": self.artifact_dir,
            "verdict": self.verdict,
            "qualified_through": "HL20-G6" if self.verdict == "qualified_through_hl20_g6" else None,
            "fidelities": list(self.fidelities),
            "contract_path": self.contract_path,
            "gates": [gate.as_dict() for gate in self.gates],
        }


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _list(value: object, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a list")
    return value


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _close(actual: float, expected: float, tolerance: float) -> bool:
    return abs(actual - expected) <= tolerance


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _display_path(path: Path) -> str:
    """Keep repository evidence portable while preserving external test paths."""

    try:
        return path.resolve().relative_to(Path.cwd().resolve()).as_posix()
    except ValueError:
        return str(path)


def _load_json(path: Path) -> dict[str, Any]:
    return _mapping(json.loads(path.read_text(encoding="utf-8")), str(path))  # type: ignore[return-value]


def _load_contract(path: Path) -> dict[str, Any]:
    return _mapping(yaml.safe_load(path.read_text(encoding="utf-8")), str(path))  # type: ignore[return-value]


def _rows(sample: Mapping[str, Any]) -> tuple[dict[str, Any], ...]:
    trajectory = _mapping(sample.get("trajectory"), "sample.trajectory")
    fields = tuple(str(field) for field in _list(trajectory.get("fields"), "trajectory.fields"))
    result: list[dict[str, Any]] = []
    for raw_row in _list(trajectory.get("rows"), "trajectory.rows"):
        if isinstance(raw_row, str):
            values: list[Any] = raw_row.split()
        elif isinstance(raw_row, list):
            values = raw_row
        else:
            raise ValueError("trajectory rows must be serialized strings or arrays")
        if len(values) != len(fields):
            raise ValueError("trajectory row width does not match trajectory fields")
        parsed: dict[str, Any] = {}
        for field, value in zip(fields, values, strict=True):
            if field == "phase":
                parsed[field] = value
            else:
                parsed[field] = float(value)
        result.append(parsed)
    return tuple(result)


def _samples(payload: Mapping[str, Any]) -> tuple[Mapping[str, Any], ...]:
    return tuple(_mapping(sample, "sample") for sample in _list(payload.get("samples"), "samples"))


def _finite_values(values: object) -> bool:
    if isinstance(values, (list, tuple)):
        return all(_finite_values(value) for value in values)
    if isinstance(values, (int, float)) and not isinstance(values, bool):
        return math.isfinite(float(values))
    return True


def _all_telemetry(samples: tuple[Mapping[str, Any], ...]) -> tuple[Mapping[str, Any], ...]:
    rows: list[Mapping[str, Any]] = []
    for sample in samples:
        rows.extend(_mapping(row, "telemetry row") for row in _list(sample.get("telemetry"), "sample.telemetry"))
    return tuple(rows)


def _all_children(samples: tuple[Mapping[str, Any], ...]) -> tuple[Mapping[str, Any], ...]:
    children: list[Mapping[str, Any]] = []
    for sample in samples:
        children.extend(_mapping(child, "spawned body") for child in _list(sample.get("spawned_bodies"), "sample.spawned_bodies"))
    return tuple(children)


def _gate(number: int, name: str, check: Any) -> HL20GateResult:
    gate_id = f"HL20-G{number}"
    try:
        message, evidence = check()
        return HL20GateResult(gate_id, number, name, "pass", message, evidence)
    except (AssertionError, KeyError, TypeError, ValueError) as error:
        return HL20GateResult(gate_id, number, name, "fail", str(error), {})


def _load_bundle(artifact_dir: Path, contract: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    manifest = _load_json(artifact_dir / "bundle-manifest.json")
    expected = tuple(str(value) for value in _list(contract.get("expected_fidelities"), "expected_fidelities"))
    payloads = {fidelity: _load_json(artifact_dir / f"{fidelity}.json") for fidelity in expected}
    return manifest, payloads


def _g0(artifact_dir: Path, contract: Mapping[str, Any], manifest: Mapping[str, Any], payloads: Mapping[str, Mapping[str, Any]]) -> tuple[str, dict[str, object]]:
    assert manifest.get("schema") == "taoryx.hl20-rocket-release-fidelity-bundle/v1alpha1", "bundle schema is not the HL-20 fidelity bundle"
    expected = tuple(str(value) for value in _list(contract["expected_fidelities"], "expected_fidelities"))
    assert tuple(manifest.get("fidelity_profiles", ())) == expected, "bundle fidelity order differs from the contract"
    assert manifest.get("vehicle_id") == contract["vehicle_id"], "bundle vehicle binding differs from the contract"
    search_spaces: set[str] = set()
    provenance_ids: list[str] = []
    artifact_hashes: dict[str, str] = {}
    for fidelity in expected:
        payload = payloads[fidelity]
        assert payload.get("schema") == contract["artifact_schema"] == _ARTIFACT_SCHEMA, f"{fidelity} artifact schema is invalid"
        study = _mapping(payload.get("study"), f"{fidelity}.study")
        provenance = _mapping(payload.get("provenance"), f"{fidelity}.provenance")
        assert study.get("fidelity") == fidelity, f"{fidelity} study fidelity is not self-identifying"
        assert provenance.get("fidelity") == fidelity, f"{fidelity} provenance fidelity is not self-identifying"
        assert study.get("vehicle_id") == "hl20-low-fidelity-release-v1", f"{fidelity} runtime vehicle binding is missing"
        assert provenance.get("family_id") == "reference_hl20_mod_k", f"{fidelity} source family binding is missing"
        assert provenance.get("source_package_sha256"), f"{fidelity} source package hash is missing"
        assert provenance.get("aerodynamics_sha256"), f"{fidelity} aerodynamics hash is missing"
        assert provenance.get("source_exact_trajectory") is False, f"{fidelity} overclaims source trajectory exactness"
        assert provenance.get("route_claim") is False, f"{fidelity} overclaims route capability"
        assert provenance.get("controller_claim") is False, f"{fidelity} overclaims controller qualification"
        search_spaces.add(json.dumps(payload.get("search_space"), sort_keys=True))
        provenance_ids.append(str(provenance.get("scenario_id")))
        artifact_hashes[fidelity] = _sha256(artifact_dir / f"{fidelity}.json")
    assert len(search_spaces) == 1, "fidelity artifacts do not share one search space"
    assert len(set(provenance_ids)) == len(expected), "fidelity provenance scenario ids are not unique"
    showcase_manifest_path = artifact_dir / "composites" / "showcase-manifest.json"
    assert showcase_manifest_path.is_file(), "showcase manifest is missing"
    showcase_manifest = _load_json(showcase_manifest_path)
    assert showcase_manifest.get("schema") == "taoryx.hl20-ca-hi-showcase-composite-manifest/v1alpha1", "showcase manifest schema is invalid"
    for source in _list(showcase_manifest.get("source_artifacts"), "showcase source_artifacts"):
        item = _mapping(source, "showcase source artifact")
        source_path = artifact_dir / "composites" / str(item["path"])
        assert source_path.is_file(), f"showcase source artifact is missing: {source_path}"
        assert _sha256(source_path) == item["sha256"], f"showcase source artifact hash mismatch: {source_path.name}"
    search_space = _mapping(payloads[expected[0]]["search_space"], "search_space")
    assert search_space.get("candidate_count") == contract["search_space"]["candidate_count"], "candidate count differs from the contract"
    return "source, assumption, artifact, and common-search-space bindings are valid", {"artifact_sha256": artifact_hashes, "provenance_scenario_ids": provenance_ids}


def _g1(contract: Mapping[str, Any], payloads: Mapping[str, Mapping[str, Any]]) -> tuple[str, dict[str, object]]:
    events = _mapping(contract["events"], "events")
    burn_time = float(events["burn_time_s"])
    release_time = float(events["release_time_s"])
    tolerance = float(events["tolerance_s"])
    phase_counts: dict[str, int] = {}
    event_times: list[float] = []
    for fidelity, payload in payloads.items():
        for sample in _samples(payload):
            rows = _rows(sample)
            assert rows, f"{fidelity} has an empty trajectory"
            times = [float(row["time_s"]) for row in rows]
            assert _close(times[0], 0.0, tolerance), f"{fidelity} trajectory does not start at t=0"
            assert all(current > previous for previous, current in zip(times[:-1], times[1:], strict=True)), f"{fidelity} trajectory time is not strictly increasing"
            phases = [str(row["phase"]) for row in rows]
            distinct = tuple(dict.fromkeys(phases))
            assert distinct == _PHASE_ORDER, f"{fidelity} phase order is {distinct}, expected {_PHASE_ORDER}"
            assert any(_close(time, burn_time, tolerance) for time in times), f"{fidelity} has no exact burnout boundary"
            assert any(_close(time, release_time, tolerance) for time in times), f"{fidelity} has no exact release boundary"
            event_list = _list(sample.get("deployment_events"), "deployment_events")
            assert len(event_list) == 1, f"{fidelity} does not have exactly one release event per candidate"
            event = _mapping(event_list[0], "deployment event")
            assert event.get("status") == "committed", f"{fidelity} release event is not committed"
            event_time = _number(event.get("accepted_time_s"), "accepted_time_s")
            assert _close(event_time, release_time, tolerance), f"{fidelity} release time is {event_time}, expected {release_time}"
            event_times.append(event_time)
            for phase in distinct:
                phase_counts[phase] = phase_counts.get(phase, 0) + 1
    return "boost, coast, and glide are ordered with exact burnout and release boundaries", {"phase_trajectory_counts": phase_counts, "release_times_s": event_times}


def _g2(contract: Mapping[str, Any], payload: Mapping[str, Any]) -> tuple[str, dict[str, object]]:
    samples = _samples(payload)
    assert payload.get("fidelity") == "point_mass_3dof", "G2 must validate the point-mass artifact"
    expected_count = int(_mapping(contract["search_space"], "search_space")["candidate_count"])
    assert len(samples) == expected_count, "3DOF candidate count does not match the contract"
    max_altitude = 0.0
    max_speed = 0.0
    for sample in samples:
        assert sample.get("classification") == "feasible", "nominal 3DOF candidate is not terminally feasible"
        assert sample.get("termination") == "ground_contact", "nominal 3DOF candidate did not terminate at ground contact"
        rows = _rows(sample)
        assert all(_finite_values(row) for row in rows), "3DOF trajectory contains a non-finite value"
        masses = [float(row["mass_kg"]) for row in rows]
        assert all(current <= previous + 1.0e-8 for previous, current in zip(masses[:-1], masses[1:], strict=True)), "3DOF mass is not non-increasing"
        max_altitude = max(max_altitude, max(float(row["z_m"]) for row in rows))
        max_speed = max(max_speed, max(float(row["speed_m_s"]) for row in rows))
        metrics = _mapping(sample.get("path_metrics"), "path_metrics")
        for key in ("duration_s", "maximum_altitude_m", "maximum_speed_m_s", "minimum_mass_kg", "terminal_specific_energy_m2_s2"):
            assert key in metrics and _finite_values(metrics[key]), f"3DOF path metric is missing: {key}"
    return "3DOF translation and resource baseline is finite, bounded, and terminally classified", {"candidate_count": len(samples), "maximum_altitude_m": max_altitude, "maximum_speed_m_s": max_speed}


def _g3(contract: Mapping[str, Any], payload: Mapping[str, Any]) -> tuple[str, dict[str, object]]:
    samples = _samples(payload)
    assert payload.get("fidelity") == "pseudo_6dof", "G3 must validate the pseudo-6DOF artifact"
    max_rate = 0.0
    for sample in samples:
        fields = tuple(str(field) for field in _list(_mapping(sample["trajectory"], "trajectory")["fields"], "trajectory.fields"))
        for field in ("roll_rad", "pitch_rad", "yaw_rad", "roll_rate_rad_s", "pitch_rate_rad_s", "yaw_rate_rad_s"):
            assert field in fields, f"pseudo-6DOF state field is missing: {field}"
        telemetry = _list(sample.get("telemetry"), "sample.telemetry")
        assert len(telemetry) == len(_rows(sample)), "pseudo-6DOF telemetry does not align with states"
        for raw_row in telemetry:
            row = _mapping(raw_row, "pseudo telemetry")
            rates = _list(row.get("attitude_rate_rad_s"), "attitude_rate_rad_s")
            assert len(rates) == 3 and all(_finite_values(rate) for rate in rates), "pseudo-6DOF attitude rates are invalid"
            max_rate = max(max_rate, *(abs(float(rate)) for rate in rates))
        for child in _all_children((sample,)):
            child_telemetry = _list(child.get("telemetry"), "child.telemetry")
            assert child_telemetry, "pseudo-6DOF child has no telemetry"
    limit = float(_mapping(contract["limits"], "limits")["max_pseudo_attitude_rate_rad_s"])
    assert max_rate <= limit, f"pseudo-6DOF attitude rate exceeds contract limit: {max_rate}"
    return "pseudo-6DOF carries aligned reduced attitude and rate channels without claiming native rotation", {"maximum_attitude_rate_rad_s": max_rate, "candidate_count": len(samples)}


def _g4(contract: Mapping[str, Any], payload: Mapping[str, Any]) -> tuple[str, dict[str, object]]:
    samples = _samples(payload)
    assert payload.get("fidelity") == "rigid_body_6dof", "G4 must validate the native rigid-body artifact"
    limits = _mapping(contract["limits"], "limits")
    max_rate = 0.0
    max_translation_residual = 0.0
    max_rotation_residual = 0.0
    for sample in samples:
        telemetry = _all_telemetry((sample,))
        assert telemetry, "rigid-body parent has no telemetry"
        for row in telemetry:
            quaternion = _list(row.get("attitude_quaternion"), "attitude_quaternion")
            assert len(quaternion) == 4 and all(_finite_values(value) for value in quaternion), "rigid-body quaternion is invalid"
            assert abs(math.sqrt(sum(float(value) ** 2 for value in quaternion)) - 1.0) <= float(limits["quaternion_norm_error"]), "rigid-body quaternion is not normalized"
            rates = _list(row.get("attitude_rate_rad_s"), "attitude_rate_rad_s")
            max_rate = max(max_rate, *(abs(float(rate)) for rate in rates))
            max_translation_residual = max(max_translation_residual, float(row["translation_equation_residual_normalized"]))
            max_rotation_residual = max(max_rotation_residual, float(row["rotation_equation_residual_normalized"]))
        children = _all_children((sample,))
        assert children, "rigid-body parent did not emit a child witness"
        child_rates = [abs(float(rate)) for child in children for row in _list(child.get("telemetry"), "child telemetry") for rate in _list(_mapping(row, "child telemetry row").get("attitude_rate_rad_s"), "child attitude_rate_rad_s")]
        assert max(child_rates, default=0.0) > 0.0, "native passive-tumble child has no observable rotation"
    assert max_rate <= float(limits["max_native_body_rate_rad_s"]), f"native body rate exceeds contract limit: {max_rate}"
    assert max_translation_residual <= float(limits["max_translation_residual_normalized"]), "native translation residual exceeds contract limit"
    assert max_rotation_residual <= float(limits["max_rotation_residual_normalized"]), "native rotation residual exceeds contract limit"
    return "native rigid-body state, equation residuals, and passive-tumble rotation are present", {"maximum_body_rate_rad_s": max_rate, "maximum_translation_residual_normalized": max_translation_residual, "maximum_rotation_residual_normalized": max_rotation_residual}


def _g5(contract: Mapping[str, Any], payloads: Mapping[str, Mapping[str, Any]]) -> tuple[str, dict[str, object]]:
    surface_payload = payloads["rigid_body_6dof_surface_allocated"]
    reference_payload = payloads["rigid_body_6dof"]
    limits = _mapping(contract["limits"], "limits")
    required = tuple(str(value) for value in _list(contract["required_surface_channels"], "required_surface_channels"))
    surface_rows = _all_telemetry(_samples(surface_payload))
    assert surface_rows, "surface-allocation tier has no telemetry"
    maximum_residual = 0.0
    for row in surface_rows:
        assert float(row["surface_allocation_active"]) == 1.0, "logical surface allocator is not marked active"
        for channel in required:
            assert channel in row and _finite_values(row[channel]), f"logical surface channel is missing: {channel}"
        for channel in ("surface_allocation_requested_bank_deg", "surface_allocation_achieved_bank_deg", "surface_allocation_requested_pitch_deg", "surface_allocation_achieved_pitch_deg", "surface_allocation_requested_yaw_deg", "surface_allocation_achieved_yaw_deg"):
            assert channel in row and _finite_values(row[channel]), f"surface allocation channel is missing: {channel}"
        maximum_residual = max(maximum_residual, abs(float(row["surface_allocation_residual_deg"])))
    assert maximum_residual <= float(limits["max_surface_residual_deg"]), "logical surface residual exceeds contract limit"
    for surface_sample, reference_sample in zip(_samples(surface_payload), _samples(reference_payload), strict=True):
        surface_rows_for_sample = _rows(surface_sample)
        reference_rows_for_sample = _rows(reference_sample)
        assert len(surface_rows_for_sample) == len(reference_rows_for_sample), "surface overlay changed trajectory sampling"
        for surface_row, reference_row in zip(surface_rows_for_sample, reference_rows_for_sample, strict=True):
            for field in ("time_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s", "mass_kg"):
                assert _close(float(surface_row[field]), float(reference_row[field]), 1.0e-8), "surface overlay changed reduced parent dynamics"
    return "logical seven-surface telemetry is bounded and proven as a non-dynamics overlay", {"telemetry_rows": len(surface_rows), "maximum_surface_residual_deg": maximum_residual, "dynamics_overlay_only": True}


def _g6(contract: Mapping[str, Any], payloads: Mapping[str, Mapping[str, Any]]) -> tuple[str, dict[str, object]]:
    events_contract = _mapping(contract["events"], "events")
    terminal = _mapping(contract["terminal"], "terminal")
    limits = _mapping(contract["limits"], "limits")
    release_time = float(events_contract["release_time_s"])
    tolerance = float(events_contract["tolerance_s"])
    parent_terminations: dict[str, int] = {}
    child_terminations: dict[str, int] = {}
    max_momentum_residual = 0.0
    deployed = 0
    timeout_ids: list[str] = []
    assert terminal.get("timeout_is_not_success") is True, "timeout policy must explicitly reject success"
    for fidelity, payload in payloads.items():
        for sample in _samples(payload):
            event_list = _list(sample.get("deployment_events"), "deployment_events")
            assert len(event_list) == 1, f"{fidelity} deployment lineage is incomplete"
            event = _mapping(event_list[0], "deployment event")
            assert event.get("event_id") == "booster-release", f"{fidelity} deployment event id is invalid"
            assert event.get("shape") == "cylinder", f"{fidelity} deployment shape is not cylinder"
            assert _close(float(event["accepted_time_s"]), release_time, tolerance), f"{fidelity} deployment time is not contract-bound"
            max_momentum_residual = max(max_momentum_residual, abs(float(event["momentum_residual_kg_m_s"])))
            children = tuple(_mapping(child, "spawned body") for child in _list(sample.get("spawned_bodies"), "spawned_bodies"))
            assert len(children) == 1, f"{fidelity} does not have exactly one spawned cylinder"
            child = children[0]
            assert child.get("body_id") == event.get("child_model_id") == "hl20-synthetic-spent-booster", f"{fidelity} child lineage id is inconsistent"
            assert child.get("parent_event_id") == event.get("event_id"), f"{fidelity} child does not reference its parent event"
            assert child.get("shape") == "cylinder", f"{fidelity} child shape is not cylinder"
            assert child.get("termination") == terminal["child_termination"], f"{fidelity} child terminal semantics are not contract-bound"
            assert child.get("telemetry"), f"{fidelity} child telemetry is missing"
            assert child.get("trajectory", {}).get("rows"), f"{fidelity} child trajectory is missing"
            parent_terminations[str(sample.get("termination"))] = parent_terminations.get(str(sample.get("termination")), 0) + 1
            child_terminations[str(child.get("termination"))] = child_terminations.get(str(child.get("termination")), 0) + 1
            deployed += 1
            if bool(sample.get("timed_out")):
                timeout_ids.append(str(sample.get("query_id")))
    if timeout_ids:
        raise ValueError(f"nominal qualification contains horizon timeouts: {timeout_ids}")
    assert max_momentum_residual <= float(limits["max_momentum_residual_kg_m_s"]), "deployment momentum residual exceeds contract limit"
    assert parent_terminations == {str(terminal["parent_termination"]): deployed}, f"parent terminal dispositions are {parent_terminations}"
    assert child_terminations == {str(terminal["child_termination"]): deployed}, f"child terminal dispositions are {child_terminations}"
    return "cylinder deployment lineage, impulse closure, child telemetry, and terminal semantics are qualified", {"deployed_children": deployed, "parent_termination_counts": parent_terminations, "child_termination_counts": child_terminations, "timed_out_query_ids": timeout_ids, "maximum_momentum_residual_kg_m_s": max_momentum_residual}


def validate_hl20_artifacts(artifact_dir: str | Path = DEFAULT_ARTIFACT_DIR, quality_gates_path: str | Path = DEFAULT_QUALITY_GATES) -> HL20QualificationReport:
    """Validate HL20-G0 through HL20-G6 over a generated bundle."""

    artifact_path = Path(artifact_dir)
    contract_path = Path(quality_gates_path)
    contract = _load_contract(contract_path)
    manifest, payloads = _load_bundle(artifact_path, contract)
    expected = tuple(str(value) for value in _list(contract["expected_fidelities"], "expected_fidelities"))
    gates = (
        _gate(0, "source and artifact contract", lambda: _g0(artifact_path, contract, manifest, payloads)),
        _gate(1, "phase and event ordering", lambda: _g1(contract, payloads)),
        _gate(2, "3DOF translation baseline", lambda: _g2(contract, payloads["point_mass_3dof"])),
        _gate(3, "pseudo-6DOF bridge", lambda: _g3(contract, payloads["pseudo_6dof"])),
        _gate(4, "native rigid-body 6DOF", lambda: _g4(contract, payloads["rigid_body_6dof"])),
        _gate(5, "logical surface telemetry overlay", lambda: _g5(contract, payloads)),
        _gate(6, "deployment lineage and terminal semantics", lambda: _g6(contract, payloads)),
    )
    verdict = "qualified_through_hl20_g6" if all(gate.status == "pass" for gate in gates) else "not_qualified"
    return HL20QualificationReport(
        schema="taoryx.hl20-qualification-report/v1alpha1",
        scenario_id=str(contract["scenario_id"]),
        vehicle_id=str(contract["vehicle_id"]),
        artifact_dir=_display_path(artifact_path),
        verdict=verdict,
        gates=gates,
        fidelities=expected,
        contract_path=_display_path(contract_path),
    )


__all__ = ["DEFAULT_ARTIFACT_DIR", "DEFAULT_QUALITY_GATES", "HL20_GATE_IDS", "HL20GateResult", "HL20QualificationReport", "validate_hl20_artifacts"]
