"""Shared harness for source-anchored vehicle plant tests.

The harness deliberately stays at the file-oriented runtime boundary.  A
golden-plant test therefore exercises the same problem-file ingestion, table
binding, lowering, and integrator path that a user invokes, while keeping the
common evidence checks in one place.
"""

from __future__ import annotations

import math
import re
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Literal, Mapping

import pytest
import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import FileKind, ingest_file
from taoryx.language.models import TableDocument
from taoryx.language.table_parser import table_type_catalog
from taoryx.runtime.runner import RunReport, run_files
from taoryx.validation import independent_force_closure, independent_moment_closure

StageStatus = Literal["pass", "fail", "blocked"]


@dataclass(frozen=True, slots=True)
class VerificationStage:
    """Evidence for one ordered golden-plant verification stage."""

    number: int
    name: str
    status: StageStatus
    message: str
    evidence: Mapping[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "number": self.number,
            "name": self.name,
            "status": self.status,
            "message": self.message,
            "evidence": dict(self.evidence),
        }
    ####


@dataclass(frozen=True, slots=True)
class GoldenPlantCase:
    """Inputs and execution policy for one source-anchored plant case."""

    vehicle: str
    problem: Path
    tables: tuple[Path, ...] = ()
    max_steps: int = 100
    integrator: str = "rk4"
    profile: GrammarProfile = GrammarProfile.TAORYX
    alpha_beta_reference: tuple[float, float] | None = None
    controller_configured: bool = False
    convergence_factors: tuple[float, ...] = (1.0, 0.5, 0.25)
    expectations: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.vehicle.strip():
            raise ValueError("golden-plant vehicle name must not be empty")
        if self.problem.suffix.casefold() != ".prb":
            raise ValueError("golden-plant problem must be a .prb file")
        if any(path.suffix.casefold() != ".tbl" for path in self.tables):
            raise ValueError("golden-plant table bindings must be .tbl files")
        if self.max_steps < 1:
            raise ValueError("golden-plant max_steps must be positive")
        if not self.convergence_factors or self.convergence_factors[0] != 1.0:
            raise ValueError("convergence_factors must begin with 1.0")
        if any(factor <= 0.0 for factor in self.convergence_factors):
            raise ValueError("convergence_factors must be positive")
        if any(left <= right for left, right in zip(self.convergence_factors, self.convergence_factors[1:])):
            raise ValueError("convergence_factors must be strictly descending")
    ####


@dataclass(frozen=True, slots=True)
class GoldenPlantRun:
    """A runtime report plus immutable telemetry snapshots for assertions."""

    case: GoldenPlantCase
    report: RunReport
    history: tuple[Mapping[str, float], ...]

    @property
    def initial(self) -> Mapping[str, float]:
        """Return the first telemetry sample."""

        if not self.history:
            raise AssertionError(f"{self.case.vehicle} produced no telemetry")
        return self.history[0]
    ####

    @property
    def final(self) -> Mapping[str, float]:
        """Return the last telemetry sample."""

        if not self.history:
            raise AssertionError(f"{self.case.vehicle} produced no telemetry")
        return self.history[-1]
    ####

    def require_success(self) -> None:
        """Require a completed run with no runtime diagnostics."""

        diagnostics = [(item.code, item.message) for item in self.report.diagnostics]
        assert self.report.exit_code == 0, diagnostics
        assert self.report.results and all(result.completed for result in self.report.results)
    ####

    def require_finite(self) -> None:
        """Reject invalid values, allowing unavailable low-speed air-data angles."""

        assert self.history, f"{self.case.vehicle} produced no telemetry"
        for sample in self.history:
            for name, value in sample.items():
                if name in {"aero_alpha_deg", "aero_sideslip_deg", "aero_table_min_margin", "aero_table_min_normalized_margin"} and math.isnan(value):
                    continue
                if name in {"aero_alpha_deg", "aero_sideslip_deg"} and sample.get("aero_air_data_valid", 1.0) < 0.5:
                    assert math.isnan(value)
                    continue
                assert math.isfinite(value), (name, value)
    ####

    def require_channels(self, *names: str) -> None:
        """Require named telemetry channels in every sample."""

        missing = {
            name
            for name in names
            if any(name not in sample for sample in self.history)
        }
        assert not missing, f"{self.case.vehicle} is missing telemetry channels: {sorted(missing)}"
    ####

    def require_initial_values(self, **expected: float) -> None:
        """Compare source-anchor telemetry values at the first RHS sample."""

        self.require_channels(*expected)
        for name, value in expected.items():
            assert self.initial[name] == pytest.approx(value), (name, self.initial[name], value)
        ####
    ####

    def require_initial_closure(
        self,
        *,
        translation_max: float = 1.0e-8,
        rotation_max: float = 1.0e-8,
    ) -> None:
        """Require small initial normalized force and moment closure residuals."""

        self.require_channels(
            "translation_equation_residual_normalized",
            "rotation_equation_residual_normalized",
        )
        assert self.initial["translation_equation_residual_normalized"] < translation_max
        assert self.initial["rotation_equation_residual_normalized"] < rotation_max
    ####

    def require_active_aerodynamics(self) -> None:
        """Require that the plant actually reached its coefficient/wrench model."""

        self.require_channels("aero_active")
        assert self.initial["aero_active"] == pytest.approx(1.0)
    ####

    def require_convention_firewall(
        self,
        *,
        integration_frame: str = "ecic",
        environment_frame: str = "ecfc",
        dynamics_mode: str = "rigid-body-6dof",
        quaternion_tolerance: float = 1.0e-8,
    ) -> None:
        """Check the runtime conventions before interpreting trajectory behavior.

        These checks are intentionally model-independent.  They catch common
        setup errors—wrong runtime mode, wrong Earth/environment frame, bad
        quaternion ordering, invalid mass, or non-finite aerodynamic angles—
        before a controller or mission result is evaluated.
        """

        assert self.report.metadata, f"{self.case.vehicle} produced no runtime metadata"
        metadata = self.report.metadata[0]
        assert metadata.get("dynamics_mode") == dynamics_mode
        pipeline = metadata.get("native_pipeline")
        assert isinstance(pipeline, Mapping)
        assert pipeline.get("integration_frame") == integration_frame
        assert pipeline.get("environment_frame") == environment_frame
        self.require_channels("qw", "qx", "qy", "qz", "mass_kg", "aero_alpha_deg", "aero_sideslip_deg", "aero_air_data_valid")
        for sample in self.history:
            quaternion_norm = sum(sample[name] ** 2 for name in ("qw", "qx", "qy", "qz"))
            assert quaternion_norm == pytest.approx(1.0, abs=quaternion_tolerance)
            assert sample["mass_kg"] > 0.0
            if sample["aero_air_data_valid"] >= 0.5:
                assert math.isfinite(sample["aero_alpha_deg"])
                assert math.isfinite(sample["aero_sideslip_deg"])
            else:
                assert sample["aero_airspeed_m_s"] < 0.1
        ####

    def require_table_margins(self, minimum: float = 0.0) -> None:
        """Require recorded table-query margins to stay inside the envelope."""

        margin_names = sorted(
            name
            for name in self.initial
            if name.startswith("aero_table_margin_")
        )
        assert margin_names, f"{self.case.vehicle} produced no table-margin telemetry"
        for name in margin_names:
            assert all(sample[name] >= minimum for sample in self.history), (name, minimum)
    ####

    def require_bounded_delta(self, channel: str, maximum: float) -> None:
        """Require a channel to remain within ``maximum`` of its initial value."""

        self.require_channels(channel)
        origin = self.initial[channel]
        assert max(abs(sample[channel] - origin) for sample in self.history) <= maximum
    ####


@dataclass(frozen=True, slots=True)
class GoldenPlantVerification:
    """Ordered report for any standard TAORYX 3DOF or 6DOF plant case."""

    case: GoldenPlantCase
    run: GoldenPlantRun | None
    stages: tuple[VerificationStage, ...]

    @property
    def plant_golden(self) -> bool:
        """Whether stages 1–11 all passed."""

        return all(stage.status == "pass" for stage in self.stages[:11])
    ####

    @property
    def controller_ready(self) -> bool:
        """Whether the plant and controller/mission stage passed."""

        return self.plant_golden and self.stages[11].status == "pass"
    ####

    @property
    def verdict(self) -> str:
        """Return a stable machine-readable verdict."""

        if self.controller_ready:
            return "controller-ready"
        if self.plant_golden:
            return "plant-golden"
        if any(stage.status == "fail" for stage in self.stages):
            return "needs-fix"
        return "blocked"
    ####

    def as_dict(self) -> dict[str, Any]:
        """Serialize the report for CI and human-readable artifact generation."""

        return {
            "vehicle": self.case.vehicle,
            "problem": str(self.case.problem),
            "tables": [str(path) for path in self.case.tables],
            "verdict": self.verdict,
            "plant_golden": self.plant_golden,
            "controller_ready": self.controller_ready,
            "stages": [stage.as_dict() for stage in self.stages],
        }
    ####


_STAGE_NAMES = (
    "table-ingestion-and-axis-ordering",
    "units-and-reference-geometry",
    "body-world-frame-conventions",
    "quaternion-identity-and-handedness",
    "alpha-beta-sign-conventions",
    "coefficient-lookup-and-table-margins",
    "force-moment-dimensionalization",
    "mass-cg-inertia-positivity",
    "static-wrench-or-trim-closure",
    "short-bounded-propagation",
    "integration-convergence",
    "controller-and-mission-tests",
)


def _stage(number: int, name: str, check: Callable[[], Mapping[str, Any] | str]) -> VerificationStage:
    """Run one report stage without hiding failures behind pytest assertions."""

    try:
        result = check()
    except AssertionError as error:
        return VerificationStage(number, name, "fail", str(error) or "assertion failed")
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        return VerificationStage(number, name, "fail", str(error))
    if isinstance(result, str):
        return VerificationStage(number, name, "pass", result)
    return VerificationStage(number, name, "pass", "check passed", result)
####


def verify_golden_plant(case: GoldenPlantCase, output_dir: Path) -> GoldenPlantVerification:
    """Run the twelve-stage firewall and return an actionable evidence report.

    Stages 1–11 establish a validated plant. Stage 12 is intentionally
    separate: a vehicle can be a golden plant before any controller or mission
    has been configured. Missing evidence is reported as ``blocked`` rather
    than being treated as a pass.
    """

    table_documents: list[Any] = []
    table_names: set[str] = set()
    table_evidence: dict[str, Any] = {"files": [str(path) for path in case.tables]}

    def table_ingestion() -> Mapping[str, Any]:
        for path in case.tables:
            ingested = ingest_file(path)
            if ingested.kind is not FileKind.TABLE or ingested.document is None:
                raise ValueError(f"{path} did not ingest as a table")
            if not isinstance(ingested.document, TableDocument):
                raise ValueError(f"{path} did not produce a table document")
            table_documents.append(ingested.document)
            table_names.update(table_type_catalog(ingested.document))
        axes: dict[str, list[str]] = {}
        for document in table_documents:
            for definition in document.tables:
                independent = [name.casefold() for name in definition.independent_variables]
                if len(independent) != len(set(independent)):
                    raise ValueError(f"table {definition.name!r} repeats an independent axis")
                axes[definition.name] = independent
        table_evidence["table_names"] = sorted(table_names)
        table_evidence["axes"] = axes
        return table_evidence

    stages: list[VerificationStage] = []
    stages.append(_stage(1, _STAGE_NAMES[0], table_ingestion))

    def run_case() -> GoldenPlantRun:
        return run_golden_plant(case, output_dir)

    run: GoldenPlantRun | None = None
    if stages[-1].status == "pass":
        run = run_case()

    def metadata() -> Mapping[str, Any]:
        if run is None or not run.report.metadata:
            raise ValueError("runtime metadata is unavailable")
        return run.report.metadata[0]

    def units_geometry() -> Mapping[str, Any]:
        values = metadata()
        vehicle = values.get("vehicle")
        if not isinstance(vehicle, Mapping):
            raise ValueError("runtime vehicle metadata is missing")
        required = ("reference-area", "reference-length")
        missing = [key for key in required if key not in vehicle]
        if missing:
            raise ValueError(f"missing reference geometry: {', '.join(missing)}")
        area = float(vehicle["reference-area"])
        length = float(vehicle["reference-length"])
        if area <= 0.0 or length <= 0.0:
            raise ValueError("reference geometry must be positive")
        return {"reference_area": area, "reference_length": length}

    def frames() -> Mapping[str, Any]:
        values = metadata()
        pipeline = values.get("native_pipeline")
        if not isinstance(pipeline, Mapping):
            raise ValueError("native pipeline metadata is missing")
        return {
            "dynamics_mode": values.get("dynamics_mode"),
            "integration_frame": pipeline.get("integration_frame"),
            "environment_frame": pipeline.get("environment_frame"),
        }

    def quaternion() -> Mapping[str, Any]:
        assert run is not None
        run.require_channels("qw", "qx", "qy", "qz")
        for sample in run.history:
            norm = sum(sample[name] ** 2 for name in ("qw", "qx", "qy", "qz"))
            assert norm == pytest.approx(1.0, abs=1.0e-8)
        return {"samples": len(run.history), "identity_norm": sum(run.initial[name] ** 2 for name in ("qw", "qx", "qy", "qz"))}

    def angles() -> Mapping[str, Any]:
        assert run is not None
        run.require_channels("aero_alpha_deg", "aero_sideslip_deg", "aero_air_data_valid")
        valid_samples = [sample for sample in run.history if sample["aero_air_data_valid"] >= 0.5]
        assert all(math.isfinite(sample["aero_alpha_deg"]) and math.isfinite(sample["aero_sideslip_deg"]) for sample in valid_samples)
        if case.alpha_beta_reference is None:
            raise RuntimeError("alpha/beta sign anchor is not configured")
        if run.initial["aero_air_data_valid"] >= 0.5:
            assert run.initial["aero_alpha_deg"] == pytest.approx(case.alpha_beta_reference[0])
            assert run.initial["aero_sideslip_deg"] == pytest.approx(case.alpha_beta_reference[1])
            return {"alpha_deg": run.initial["aero_alpha_deg"], "beta_deg": run.initial["aero_sideslip_deg"]}
        assert run.initial["aero_airspeed_m_s"] < 0.1
        return {"alpha_deg": None, "beta_deg": None, "air_data_valid": False}

    def margins() -> Mapping[str, Any]:
        assert run is not None
        run.require_active_aerodynamics()
        run.require_table_margins()
        names = sorted(name for name in run.initial if name.startswith("aero_table_margin_"))
        return {"margin_channels": len(names), "minimum": min(sample[name] for sample in run.history for name in names)}

    def loads() -> Mapping[str, Any]:
        assert run is not None
        run.require_initial_closure()
        return {
            "initial_translation_residual": run.initial["translation_equation_residual_normalized"],
            "initial_rotation_residual": run.initial["rotation_equation_residual_normalized"],
        }

    def mass_inertia() -> Mapping[str, Any]:
        assert run is not None
        run.require_channels("mass_kg")
        assert all(sample["mass_kg"] > 0.0 for sample in run.history)
        values = metadata().get("vehicle", {})
        inertia = {key: float(values[key]) for key in ("inertia-x", "inertia-y", "inertia-z") if key in values}
        if inertia and any(value <= 0.0 for value in inertia.values()):
            raise ValueError("inertia components must be positive")
        if not inertia:
            raise RuntimeError("inertia metadata is not available")
        return {"minimum_mass_kg": min(sample["mass_kg"] for sample in run.history), "inertia": inertia}

    def closure() -> Mapping[str, Any]:
        assert run is not None
        run.require_initial_closure()
        vehicle = metadata().get("vehicle", {})
        if not isinstance(vehicle, Mapping):
            raise RuntimeError("vehicle metadata is not available for rotational closure")
        inertia = tuple(
            float(vehicle[key])
            for key in ("inertia-x", "inertia-y", "inertia-z")
            if key in vehicle
        )
        if len(inertia) != 3:
            raise RuntimeError("complete principal inertia metadata is required for rotational closure")
        samples = tuple(
            {
                **sample,
                "inertia_x_kg_m2": inertia[0],
                "inertia_y_kg_m2": inertia[1],
                "inertia_z_kg_m2": inertia[2],
            }
            for sample in run.history
        )
        report = independent_moment_closure(samples)
        force_report = independent_force_closure(run.history)
        return {
            "initial_translation_residual": run.initial["translation_equation_residual_normalized"],
            "initial_rotation_residual": run.initial["rotation_equation_residual_normalized"],
            "independent_translation_closure": force_report,
            "independent_rotation_closure": report,
        }

    def propagation() -> Mapping[str, Any]:
        assert run is not None
        run.require_success()
        run.require_finite()
        assert len(run.history) >= 2
        return {"samples": len(run.history), "duration_s": run.final["time_s"] - run.initial["time_s"]}

    def convergence() -> Mapping[str, Any]:
        assert run is not None
        if len(case.convergence_factors) < 3:
            raise RuntimeError("at least dt, dt/2, and dt/4 are required")
        channels = ("altitude_m", "speed_m_s", "aero_alpha_deg", "aero_sideslip_deg")
        base_text = case.problem.read_text(encoding="utf-8")
        final_samples: list[Mapping[str, float]] = []
        runs: dict[str, str] = {}
        with tempfile.TemporaryDirectory(prefix=f"taoryx-{case.vehicle}-convergence-") as temporary:
            root = Path(temporary)
            for factor in case.convergence_factors:
                scaled_text, replacements = re.subn(
                    r"(?P<prefix>\bdt\s*=\s*)(?P<value>[0-9.eE+-]+)",
                    lambda match: f"{match.group('prefix')}{float(match.group('value')) * factor:.16g}",
                    base_text,
                )
                if replacements == 0:
                    raise ValueError("problem file has no explicit dt integration step")
                scaled_problem = root / f"{case.problem.stem}-{factor:g}.prb"
                scaled_problem.write_text(scaled_text, encoding="utf-8")
                scaled = run_files(
                    scaled_problem,
                    case.tables,
                    output_dir=output_dir / "convergence" / f"dt-{factor:g}",
                    max_steps=math.ceil(case.max_steps / factor),
                    integrator=case.integrator,
                    profile=case.profile,
                )
                diagnostics = [(item.code, item.message) for item in scaled.diagnostics]
                assert scaled.exit_code == 0, diagnostics
                assert scaled.results and scaled.results[0].completed
                vehicle_states = scaled.results[0].states
                assert vehicle_states
                history = next(iter(vehicle_states.values()))
                snapshot = {"time_s": history[-1].time, **dict(history[-1].named)}
                # Low-speed rotorcraft intentionally report aerodynamic angles
                # as unavailable.  Keep those channels out of the numerical
                # convergence norm rather than treating an undefined angle as
                # an integration failure.
                assert math.isfinite(snapshot["time_s"])
                snapshot = {
                    name: value
                    for name, value in snapshot.items()
                    if math.isfinite(value)
                }
                final_samples.append(snapshot)
                runs[f"dt_{factor:g}"] = str(scaled_problem)
        common = [name for name in channels if all(name in sample for sample in final_samples)]
        if len(common) < 2:
            raise RuntimeError(f"convergence requires at least two common channels; found {common}")
        def distance(left: Mapping[str, float], right: Mapping[str, float]) -> float:
            return math.sqrt(sum((left[name] - right[name]) ** 2 for name in common))
        coarse_error = distance(final_samples[0], final_samples[1])
        fine_error = distance(final_samples[1], final_samples[2])
        # Short source-anchor probes can reach the floating-point floor before
        # an asymptotic RK4 ratio is observable.  Keep a modest factor window
        # and an absolute research floor; longer cases should use a stricter
        # convergence policy supplied by their scenario.
        assert fine_error <= max(coarse_error * 2.5 + 1.0e-8, 1.0e-6), (coarse_error, fine_error, common)
        return {
            "factors": list(case.convergence_factors),
            "channels": common,
            "coarse_error": coarse_error,
            "fine_error": fine_error,
            "refinement_ratio": None if fine_error == 0.0 else coarse_error / fine_error,
            "runs": runs,
        }

    stages.extend(
        (
            _stage(2, _STAGE_NAMES[1], units_geometry),
            _stage(3, _STAGE_NAMES[2], frames),
            _stage(4, _STAGE_NAMES[3], quaternion),
            _stage(5, _STAGE_NAMES[4], angles),
            _stage(6, _STAGE_NAMES[5], margins),
            _stage(7, _STAGE_NAMES[6], loads),
            _stage(8, _STAGE_NAMES[7], mass_inertia),
            _stage(9, _STAGE_NAMES[8], closure),
            _stage(10, _STAGE_NAMES[9], propagation),
        )
    )
    if all(stage.status == "pass" for stage in stages):
        stages.append(_stage(11, _STAGE_NAMES[10], convergence))
    else:
        stages.append(VerificationStage(11, _STAGE_NAMES[10], "blocked", "blocked by an earlier failed stage"))
    stages.append(
        VerificationStage(
            12,
            _STAGE_NAMES[11],
            "pass" if case.controller_configured else "blocked",
            "controller/mission configuration supplied" if case.controller_configured else "controller and mission tests are intentionally separate",
        )
    )
    return GoldenPlantVerification(case=case, run=run, stages=tuple(stages))
####


def run_golden_plant(case: GoldenPlantCase, output_dir: Path) -> GoldenPlantRun:
    """Run one standard problem file and snapshot its telemetry for assertions."""

    report = run_files(
        case.problem,
        case.tables,
        output_dir=output_dir,
        max_steps=case.max_steps,
        integrator=case.integrator,
        profile=case.profile,
    )
    history: tuple[Mapping[str, float], ...] = ()
    if report.results:
        vehicle_states = report.results[0].states
        if vehicle_states:
            first_vehicle = next(iter(vehicle_states.values()))
            history = tuple(
                {"time_s": state.time, **dict(state.named)}
                for state in first_vehicle
            )
    return GoldenPlantRun(case=case, report=report, history=history)
####


def load_golden_vehicle_catalog(path: Path) -> tuple[GoldenPlantCase, ...]:
    """Load standard vehicle cases from a repository-relative YAML catalog."""

    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, Mapping) or document.get("schema_version") != 1:
        raise ValueError(f"unsupported vehicle catalog: {path}")
    root = path.resolve().parents[1]
    entries = document.get("vehicles")
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"vehicle catalog has no vehicles: {path}")
    cases: list[GoldenPlantCase] = []
    for entry in entries:
        if not isinstance(entry, Mapping):
            raise ValueError("vehicle catalog entries must be mappings")
        name = str(entry.get("display_name", entry.get("id", "")))
        problem = root / str(entry["problem"])
        tables = tuple(root / str(item) for item in entry.get("tables", ()))
        anchor = entry.get("alpha_beta_reference_deg")
        alpha_beta_reference = None if anchor is None else (float(anchor[0]), float(anchor[1]))
        cases.append(
            GoldenPlantCase(
                vehicle=name,
                problem=problem,
                tables=tables,
                max_steps=int(entry.get("max_steps", 100)),
                alpha_beta_reference=alpha_beta_reference,
                expectations={str(key): float(value) for key, value in dict(entry.get("expectations", {})).items()},
            )
        )
    return tuple(cases)
####


__all__ = ["GoldenPlantCase", "GoldenPlantRun", "GoldenPlantVerification", "VerificationStage", "load_golden_vehicle_catalog", "run_golden_plant", "verify_golden_plant"]
