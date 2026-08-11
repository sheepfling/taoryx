"""Public batch execution for local named-coordinate LQI controller screens."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .claim_bound_evidence import bind_release_evidence
from .composition_control_trace import (
    build_uncontrolled_committed_control_trace,
    control_trace_summary,
)
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .local_native_coordinate_lqi import (
    LocalNativeCoordinateLqiScreenExecution,
    apply_tuning_context_to_native_coordinate_lqi_config,
    run_local_native_coordinate_lqi_screen,
)
from .local_native_coordinate_lqi_mission_translation import (
    LocalNativeCoordinateLqiScreenMissionPlan,
    compile_local_native_coordinate_lqi_screen_mission,
)
from .local_native_coordinate_lqi_screen_registry import resolve_local_native_coordinate_lqi_screen_definition
from .tuning_application import RuntimeTuningBindingReceipt, TuningApplicationContext
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition


@dataclass(frozen=True, slots=True)
class LocalNativeCoordinateLqiCompositionExecution:
    """One first-class Composition packet for a bounded native-coordinate LQI screen."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: LocalNativeCoordinateLqiScreenMissionPlan
    output_dir: Path
    screen: LocalNativeCoordinateLqiScreenExecution
    mission_graph_execution: dict[str, object]
    status_trace: dict[str, object]
    semantic_action_trace: dict[str, object]
    claim_boundary: str
    tuning_binding: RuntimeTuningBindingReceipt | None = None

    @property
    def screen_pass(self) -> bool:
        """Return the local recovery verdict, not a route outcome."""

        return self.screen.screen_pass
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a compact public result while preserving the control boundary."""

        return {
            "schema": "taoryx.local-native-coordinate-lqi-composition-execution/v1alpha1",
            "status": "development_local_screen_pass" if self.screen_pass else "development_local_screen_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": {
                "controller_method": "lqi",
                "control_realization": "native_named_coordinates",
                "physical_effector_allocation": False,
                "integral_output_names": list(
                    self.screen.config.candidate.lqi.output_names if self.screen.config.candidate.lqi else ()
                ),
                "integrators_exercised": self.screen.validation.integrators_exercised,
                "tuning_binding": None if self.tuning_binding is None else self.tuning_binding.as_dict(),
            },
            "control_screen": {
                "screen_pass": self.screen_pass,
                "initial_error_norm": self.screen.initial_error_norm,
                "final_error_norm": self.screen.final_error_norm,
                "final_error_fraction": self.screen.final_error_fraction,
                "controller_method": "lqi",
                "integral_output_names": list(self.screen.config.candidate.lqi.output_names if self.screen.config.candidate.lqi else ()),
                "integrators_exercised": self.screen.validation.integrators_exercised,
                "control_realization": "native_named_coordinates",
                "physical_effector_allocation": False,
                "semantic_action_trace": "emits_committed_interval_trace_without_batch_visible_actions",
            },
            "mission_graph_execution": self.mission_graph_execution,
            "status_trace": status_trace_summary(self.status_trace),
            "semantic_action_trace": control_trace_summary(self.semantic_action_trace),
            "claim_boundary": self.claim_boundary,
        }
        ####
    ####


def execute_local_native_coordinate_lqi_composition(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    *,
    tuning_context: TuningApplicationContext | None = None,
) -> LocalNativeCoordinateLqiCompositionExecution:
    """Run exactly one registered native-coordinate LQI screen without fallback."""

    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no native-coordinate LQI translator"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    definition = resolve_local_native_coordinate_lqi_screen_definition(composition)
    if definition is None:
        raise ValueError("no local native-coordinate LQI screen factory is registered for this composition")
    config = definition.config_factory()
    tuning_binding = None
    if tuning_context is not None:
        config, tuning_binding = apply_tuning_context_to_native_coordinate_lqi_config(config, tuning_context)
    plan = compile_local_native_coordinate_lqi_screen_mission(
        composition,
        family_id=definition.family_id,
        mission_id=definition.mission_id,
        fidelity=definition.fidelity,
        initialization_id=definition.initialization_id,
        segment_id=definition.segment_id,
        screen_config_id=config.id,
    )
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    screen = run_local_native_coordinate_lqi_screen(config)
    mission_graph_execution = unobserved_mission_graph_execution(
        composition,
        "The local native-coordinate LQI screen has no route graph dispatcher; it is not a navigation execution.",
    ).as_dict()
    status_trace = build_committed_status_trace(composition, _status_samples(screen))
    semantic_action_trace = _native_coordinate_control_trace(composition, screen)
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    result = LocalNativeCoordinateLqiCompositionExecution(
        composition,
        preflight,
        plan,
        destination,
        screen,
        mission_graph_execution,
        status_trace,
        semantic_action_trace,
        definition.claim_boundary,
        tuning_binding,
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    evaluation = _screen_evaluation(screen)
    _write_json(destination / "local_screen.json", screen.as_dict())
    _write_json(destination / "truth_telemetry.json", _truth_rows(screen))
    _write_json(destination / "objective_report.json", evaluation)
    _write_json(
        destination / "convergence_report.json",
        bind_release_evidence(
            _convergence_report(screen),
            kind="convergence",
            composition=composition,
        ),
    )
    _write_json(destination / "mission_graph_execution.json", mission_graph_execution)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "semantic_action_trace.json", semantic_action_trace)
    _write_json(destination / "resource_ledger.json", resource_ledger)
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _status_samples(screen: LocalNativeCoordinateLqiScreenExecution) -> tuple[BatchTruthSample, ...]:
    """Use the plug-in's exact status mapper for every committed LQI sample."""

    values: list[BatchTruthSample] = []
    for index, sample in enumerate(screen.validation.samples):
        raw = screen.config.status_sample_mapper(sample)
        if not isinstance(raw, Mapping):
            raise TypeError("native-coordinate LQI status mapper must return a mapping")
        values.append(
            BatchTruthSample(
                time_s=sample.time_s,
                execution_status="completed" if index == len(screen.validation.samples) - 1 else "active",
                raw_values={**screen.config.resource_values, **raw, "controller_method": "lqi"},
            )
        )
    if not values:
        raise ValueError("native-coordinate LQI screen produced no committed samples")
    return tuple(values)
    ####


def _truth_rows(screen: LocalNativeCoordinateLqiScreenExecution) -> list[dict[str, object]]:
    """Emit the source-owned status projection beside each LQI sample.

    Native-coordinate validation samples retain the controller's generic
    ``state`` and named-control nesting.  Those names alone are not the
    vehicle's advertised output vocabulary, though: the plug-in's status
    mapper is the explicit, source-owned projection that defines local
    coordinates, performance values, and resources for this screen.  Keep
    both forms in the truth artifact so the standardized result reader can
    resolve only the advertised values without inventing a route or an
    effector allocation.
    """

    rows: list[dict[str, object]] = []
    for sample in screen.validation.samples:
        raw = screen.config.status_sample_mapper(sample)
        if not isinstance(raw, Mapping):
            raise TypeError("native-coordinate LQI status mapper must return a mapping")
        feedback_error_norm = math.sqrt(sum(float(value) ** 2 for value in sample.state_error.values()))
        rows.append(
            {
                **sample.as_dict(),
                **{str(name): value for name, value in screen.config.resource_values.items()},
                **dict(raw),
                "feedback_error_norm": feedback_error_norm,
                "saturation_count": len(sample.control_saturated),
            }
        )
    if not rows:
        raise ValueError("native-coordinate LQI screen produced no truth samples")
    return rows
    ####


def _native_coordinate_control_trace(
    composition: CompiledVehicleComposition,
    screen: LocalNativeCoordinateLqiScreenExecution,
) -> dict[str, object]:
    """Record committed intervals without misrepresenting native controls as API actions.

    The A320 LQI screen exposes named controls in its output telemetry, but
    does not accept caller-provided actions or declare physical effectors.
    An empty, identity-bound control trace therefore records the exact sample
    boundaries and makes that absence inspectable instead of treating the
    screen as though it forgot its action history.
    """

    return build_uncontrolled_committed_control_trace(
        composition,
        [sample.time_s for sample in screen.validation.samples],
    )
    ####


def _screen_evaluation(screen: LocalNativeCoordinateLqiScreenExecution) -> dict[str, object]:
    """Publish the bounded local recovery gates in the common objective shape."""

    requirements = [
        {
            "id": "closed_loop_candidate_safe",
            "actual": screen.config.candidate.status,
            "limit": "safe",
            "passed": screen.config.candidate.safe,
        },
        {
            "id": "final_error_fraction",
            "actual": screen.final_error_fraction,
            "limit": screen.config.final_error_fraction_limit,
            "passed": screen.final_error_fraction < screen.config.final_error_fraction_limit,
        },
        {
            "id": "control_saturation_fraction",
            "actual": screen.validation.control_saturation_fraction,
            "limit": screen.config.maximum_control_saturation_fraction,
            "passed": screen.validation.control_saturation_fraction <= screen.config.maximum_control_saturation_fraction,
        },
        {
            "id": "integrators_exercised",
            "actual": screen.validation.integrators_exercised,
            "passed": screen.validation.integrators_exercised,
        },
    ]

    return {
        "schema": "taoryx.local-native-coordinate-lqi-screen-evaluation/v1alpha1",
        "kind": "local_controller_recovery_screen",
        "controller_method": "lqi",
        "screen_pass": screen.screen_pass,
        "mission_pass": screen.screen_pass,
        "results": [
            {
                "id": str(requirement["id"]),
                "required": True,
                "status": "pass" if requirement["passed"] else "fail",
                "actual": requirement["actual"],
                **({"limit": requirement["limit"]} if "limit" in requirement else {}),
            }
            for requirement in requirements
        ],
        "requirements": requirements,
        "claim_boundary": (
            "The result assesses one pinned local named-coordinate LQI recovery only. It is not a route, "
            "persistent-disturbance, physical-effector, or qualification evaluation."
        ),
    }
    ####


def _convergence_report(screen: LocalNativeCoordinateLqiScreenExecution) -> dict[str, object]:
    """Bind the nominal local recovery result as a release-grade convergence sidecar."""

    return {
        "schema": "taoryx.endpoint-convergence-screen/v1alpha1",
        "id": screen.config.id,
        "kind": "local_controller_recovery",
        "status": "pass" if screen.screen_pass else "fail",
        "pass": screen.screen_pass,
        "controller_method": "lqi",
        "control_realization": "native_named_coordinates",
        "metrics": {
            "final_error_fraction": screen.final_error_fraction,
            "control_saturation_fraction": screen.validation.control_saturation_fraction,
            "integrators_exercised": screen.validation.integrators_exercised,
        },
        "claim_boundary": (
            "This sidecar binds one nominal local named-coordinate LQI recovery to the compiled composition. "
            "It does not establish a persistent-disturbance, mass, wind, physical-effector, route, or qualification claim."
        ),
    }
    ####


def _write_json(path: Path, payload: object) -> None:
    """Write deterministic JSON artifacts for the public packet."""

    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


__all__ = ["LocalNativeCoordinateLqiCompositionExecution", "execute_local_native_coordinate_lqi_composition"]
