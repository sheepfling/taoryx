from __future__ import annotations

from taoryx.outputs import DynamicsKind, RunArtifact, TelemetryChannel, VehicleKind, VehicleTelemetry
from taoryx.simulation_runtime_catalog import load_simulation_runtime_catalog
from taoryx.simulation_runtime_quality import (
    SimulationRuntimeInvariantSpec,
    evaluate_simulation_runtime_quality,
    validate_simulation_runtime_quality_catalog,
)


def _artifact(*, signal: tuple[float, ...] = (0.0, 1.0, 2.0), model_fingerprint: str | None = None) -> RunArtifact:
    times = [float(index) for index in range(len(signal))]
    channels = {
        "signal": TelemetryChannel(source_name="signal", semantic_name="signal", values=list(signal)),
        "mass.total": TelemetryChannel(source_name="mass", semantic_name="mass.total", values=[10.0, 9.0, 8.0]),
        "propulsion.mass_flow": TelemetryChannel(source_name="mdot", semantic_name="propulsion.mass_flow", values=[1.0, 1.0, 1.0]),
    }
    visualization = {} if model_fingerprint is None else {"model_fingerprint": model_fingerprint}
    return RunArtifact(
        problem="quality-fixture",
        vehicles={
            "vehicle": VehicleTelemetry(
                vehicle_id="vehicle",
                name="fixture",
                kind=VehicleKind.GENERIC,
                dynamics=DynamicsKind.POINT_MASS_3DOF,
                times=times,
                channels=channels,
            )
        },
        visualization=visualization,
    )
    ####


def test_simulation_runtime_catalog_declares_isolated_lanes_and_refinement_cases() -> None:
    catalog = load_simulation_runtime_catalog()

    assert validate_simulation_runtime_quality_catalog(catalog.scenarios) == ()
    assert [scenario.id for scenario in catalog.scenarios if scenario.quality.refinement.enabled] == [
        "two-stage-ballistic",
        "two-stage-demo",
    ]
    assert catalog.find("nesc-pseudo6dof").quality.fidelity.response_law
    assert catalog.find("x15-staged-pseudo6dof").quality.fidelity.lane == "x15"
    assert catalog.find("hl20-source-release-pseudo6dof").quality.fidelity.lane == "hl20"
    assert catalog.find("interactive-california-hawaii").quality.fidelity.lane == "synthetic-ca-hi"
    ####


def test_simulation_runtime_quality_passes_repeatability_refinement_and_declared_mass_balance() -> None:
    source = load_simulation_runtime_catalog().find("two-stage-ballistic")
    quality = source.quality.model_copy(
        update={
            "finite_channels": ("signal", "mass.total", "propulsion.mass_flow"),
            "continuity_channels": {"signal": 2.0},
            "repeatability": source.quality.repeatability.model_copy(update={"channels": ("signal",)}),
            "refinement": source.quality.refinement.model_copy(
                update={"channels": ("signal",), "max_absolute_error": 0.25, "max_event_time_error_s": 1.0}
            ),
            "invariants": (
                SimulationRuntimeInvariantSpec(
                    id="mass",
                    kind="mass_balance",
                    tolerance=1.0e-12,
                    mass_channel="mass.total",
                    mass_rate_channel="propulsion.mass_flow",
                ),
            ),
        }
    )
    scenario = source.model_copy(update={"quality": quality})
    first = _artifact(model_fingerprint="fixture-model")
    second = _artifact(model_fingerprint="fixture-model")
    coarse = _artifact(signal=(0.0, 1.1, 2.1), model_fingerprint="fixture-model")
    fine = _artifact(signal=(0.0, 1.0, 2.0), model_fingerprint="fixture-model")

    report = evaluate_simulation_runtime_quality(scenario, primary=first, repeat=second, coarse=coarse, fine=fine)

    assert report.disposition == "pass"
    assert {check.id: check.disposition for check in report.checks}["repeatability"] == "pass"
    assert {check.id: check.disposition for check in report.checks}["refinement"] == "pass"
    assert {check.id: check.disposition for check in report.checks}["invariant-mass"] == "pass"
    ####


def test_simulation_runtime_quality_rejects_nonfinite_declared_telemetry_and_identity_drift() -> None:
    source = load_simulation_runtime_catalog().find("two-stage-ballistic")
    quality = source.quality.model_copy(
        update={
            "finite_channels": ("signal",),
            "continuity_channels": {},
            "repeatability": source.quality.repeatability.model_copy(update={"channels": ("signal",)}),
            "refinement": source.quality.refinement.model_copy(update={"enabled": False}),
        }
    )
    scenario = source.model_copy(update={"quality": quality})
    first = _artifact(signal=(0.0, float("nan"), 2.0), model_fingerprint="model-a")
    second = _artifact(model_fingerprint="model-b")

    report = evaluate_simulation_runtime_quality(scenario, primary=first, repeat=second)

    checks = {check.id: check for check in report.checks}
    assert report.disposition == "bounded-failure"
    assert checks["finite-telemetry"].disposition == "bounded-failure"
    assert checks["repeatability"].disposition == "bounded-failure"
    ####
