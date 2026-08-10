"""Pluggable controller-design metadata and factories.

The plant, trim, and allocator contracts are shared.  This module records
which synthesis method is being used and provides the first concrete factory
for LQR.  Other methods can be added without changing vehicle problem files.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Literal, cast

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .controller_realization import (
    ClosedLoopPole,
    ControllerChannel,
    ControllerControlPath,
    ControllerEvidenceTier,
    ControllerFidelity,
    ControllerRealization,
    ControllerRole,
)
from .runtime.lqr import LqiController, LqrController, solve_continuous_lqi, solve_continuous_lqr, solve_scaled_continuous_lqi
from .trim import TrimResult

ControllerDesignMethod = Literal[
    "lqr",
    "lqi",
    "pid",
    "mpc",
    "pole_placement",
    "dynamic_inversion",
    "rule_based",
]


class ControllerDesignSpec(BaseModel):
    """Vehicle-independent controller design declaration.

    ``trim`` identifies the operating-point artifact and ``allocator`` names
    the vehicle contract that applies actuator limits and mappings.  The
    method is metadata until a matching factory is requested.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    method: ControllerDesignMethod
    trim: str = Field(min_length=1)
    allocator: str = Field(min_length=1)
    states: tuple[str, ...] = ()
    controls: tuple[str, ...] = ()
    role: str = "unspecified"
    implementation_version: str = "unversioned"
    fidelity: str = "rigid_body_6dof"
    state_units: tuple[str, ...] = ()
    state_frames: tuple[str, ...] = ()
    control_units: tuple[str, ...] = ()
    control_frames: tuple[str, ...] = ()
    plant_source: str = "unspecified"
    linearization_source: str = "unspecified"
    q_id: str = "unspecified"
    r_id: str = "unspecified"
    state_scale_id: str = "unspecified"
    control_scale_id: str = "unspecified"
    fallback_controller_id: str | None = None
    scenario_overrides_allowed: bool = False
    evidence_tier: ControllerEvidenceTier = "T0_structural"
    control_realization_path: ControllerControlPath = "unspecified"
    integral_outputs: tuple[str, ...] = ()
    screen_only: bool = False
    notes: str = ""

    @model_validator(mode="after")
    def validate_channels(self) -> ControllerDesignSpec:
        if len(set(self.states)) != len(self.states):
            raise ValueError(f"controller design {self.id!r} has duplicate states")
        if len(set(self.controls)) != len(self.controls):
            raise ValueError(f"controller design {self.id!r} has duplicate controls")
        if set(self.states) & set(self.controls):
            raise ValueError(f"controller design {self.id!r} overlaps state and control names")
        if self.method == "lqi" and not self.integral_outputs:
            raise ValueError(f"LQI controller design {self.id!r} requires integral outputs")
        if set(self.integral_outputs) - set(self.states):
            raise ValueError(f"controller design {self.id!r} integral outputs must identify declared states")
        if len(set(self.integral_outputs)) != len(self.integral_outputs):
            raise ValueError(f"controller design {self.id!r} has duplicate integral outputs")
        for values, label, expected in (
            (self.state_units, "state_units", len(self.states)),
            (self.state_frames, "state_frames", len(self.states)),
            (self.control_units, "control_units", len(self.controls)),
            (self.control_frames, "control_frames", len(self.controls)),
        ):
            if values and len(values) != expected:
                raise ValueError(f"controller design {self.id!r} {label} must match its channel count")
        if self.control_realization_path in {"direct_wrench_screen", "unconstrained_effector_allocation"} and not self.screen_only:
            raise ValueError(
                f"controller design {self.id!r} must mark {self.control_realization_path!r} as screen_only"
            )
        if self.screen_only and self.evidence_tier in {
            "T4_physically_allocated",
            "T5_nonlinearly_validated",
            "T6_envelope_validated",
        }:
            raise ValueError("screen_only controller designs cannot claim physical-effector evidence")
        return self
        ####
    ####


class ControllerDesignCatalog(BaseModel):
    """Validated collection of controller design declarations."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(ge=1)
    id: str = Field(min_length=1)
    designs: tuple[ControllerDesignSpec, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def unique_ids(self) -> ControllerDesignCatalog:
        ids = [design.id for design in self.designs]
        if len(set(ids)) != len(ids):
            raise ValueError("controller design IDs must be unique")
        return self
        ####
    ####

    def get(self, identifier: str) -> ControllerDesignSpec:
        """Return one design declaration by stable identifier."""

        for design in self.designs:
            if design.id == identifier:
                return design
        raise KeyError(f"unknown controller design {identifier!r}")
        ####


def load_controller_catalog(path: Path) -> ControllerDesignCatalog:
    """Load a YAML controller-design catalog."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"controller design catalog must be a mapping: {path}")
    return ControllerDesignCatalog.model_validate(payload)
    ####


def build_lqr_controller(
    design: ControllerDesignSpec,
    trim: TrimResult,
    a: Sequence[Sequence[float]],
    b: Sequence[Sequence[float]],
    q: Sequence[Sequence[float]],
    r: Sequence[Sequence[float]],
    *,
    lower: Mapping[str, float] | None = None,
    upper: Mapping[str, float] | None = None,
    state_adapter: Callable[[Mapping[str, float]], Mapping[str, float]] | None = None,
) -> LqrController:
    """Build an LQR controller from a solved, named plant trim.

    This is intentionally the only LQR-specific entry point.  The trim's
    state/control names are checked against the design declaration before the
    gain is created, preventing accidental reuse of gains across vehicles.
    """

    if design.method != "lqr":
        raise ValueError(f"controller design {design.id!r} uses method {design.method!r}, not 'lqr'")
    if tuple(design.states) != tuple(trim.spec.state_names):
        raise ValueError(f"controller design {design.id!r} states do not match trim {design.trim!r}")
    if tuple(design.controls) != tuple(trim.spec.control_names):
        raise ValueError(f"controller design {design.id!r} controls do not match trim {design.trim!r}")
    result = solve_continuous_lqr(
        a,
        b,
        q,
        r,
        state_names=design.states,
        control_names=design.controls,
    )
    realization = _build_lqr_realization(design, trim, result)
    return LqrController(
        result,
        state_trim=trim.state,
        control_trim=trim.controls,
        lower=lower or {},
        upper=upper or {},
        realization=realization,
        state_adapter=state_adapter,
    )
    ####


def build_lqi_controller(
    design: ControllerDesignSpec,
    trim: TrimResult,
    a: Sequence[Sequence[float]],
    b: Sequence[Sequence[float]],
    q: Sequence[Sequence[float]],
    r: Sequence[Sequence[float]],
    *,
    output_matrix: Sequence[Sequence[float]],
    lower: Mapping[str, float] | None = None,
    upper: Mapping[str, float] | None = None,
    integral_lower: Mapping[str, float] | None = None,
    integral_upper: Mapping[str, float] | None = None,
    state_scales: Sequence[float] | None = None,
    control_scales: Sequence[float] | None = None,
    state_adapter: Callable[[Mapping[str, float]], Mapping[str, float]] | None = None,
) -> LqiController:
    """Build an explicit output-integrating LQI controller from a named trim.

    ``integral_outputs`` selects the physical state outputs whose persistent
    tracking error is integrated.  The caller supplies ``output_matrix`` so a
    nontrivial measured output cannot be guessed from a state name.
    """

    if design.method != "lqi":
        raise ValueError(f"controller design {design.id!r} uses method {design.method!r}, not 'lqi'")
    if tuple(design.states) != tuple(trim.spec.state_names):
        raise ValueError(f"controller design {design.id!r} states do not match trim {design.trim!r}")
    if tuple(design.controls) != tuple(trim.spec.control_names):
        raise ValueError(f"controller design {design.id!r} controls do not match trim {design.trim!r}")
    solver = solve_continuous_lqi
    kwargs: dict[str, object] = {}
    if state_scales is not None or control_scales is not None:
        if state_scales is None or control_scales is None:
            raise ValueError("scaled LQI requires both state_scales and control_scales")
        solver = solve_scaled_continuous_lqi
        kwargs = {"state_scales": state_scales, "control_scales": control_scales}
    result = solver(
        a,
        b,
        q,
        r,
        output_matrix=output_matrix,
        output_names=design.integral_outputs,
        state_names=design.states,
        control_names=design.controls,
        **kwargs,
    )
    realization = _build_lqi_realization(design, trim, result)
    import numpy as np

    trim_state = np.asarray([float(trim.state[name]) for name in design.states], dtype=float)
    output_trim = np.asarray(output_matrix, dtype=float) @ trim_state
    return LqiController(
        result,
        state_trim=trim.state,
        control_trim=trim.controls,
        output_trim={name: float(value) for name, value in zip(design.integral_outputs, output_trim, strict=True)},
        lower=lower or {},
        upper=upper or {},
        integral_lower=integral_lower or {},
        integral_upper=integral_upper or {},
        realization=realization,
        state_adapter=state_adapter,
    )
    ####


def _build_lqr_realization(design: ControllerDesignSpec, trim: TrimResult, result: object) -> ControllerRealization:
    """Build the immutable runtime contract attached to a factory result."""

    from .runtime.lqr import LqrResult

    if not isinstance(result, LqrResult):
        raise TypeError("LQR realization requires an LqrResult")
    state_units = design.state_units or tuple("unspecified" for _ in design.states)
    state_frames = design.state_frames or tuple("unspecified" for _ in design.states)
    control_units = design.control_units or tuple("unspecified" for _ in design.controls)
    control_frames = design.control_frames or tuple("unspecified" for _ in design.controls)
    channels = tuple(
        ControllerChannel(name=name, order=index, unit=state_units[index], frame=state_frames[index], scale=1.0)
        for index, name in enumerate(design.states)
    )
    inputs = tuple(
        ControllerChannel(name=name, order=index, unit=control_units[index], frame=control_frames[index], scale=1.0)
        for index, name in enumerate(design.controls)
    )
    poles = tuple(ClosedLoopPole(real=float(value.real), imaginary=float(value.imag)) for value in result.closed_loop_eigenvalues)
    return ControllerRealization(
        id=f"{design.id}:realization",
        role=cast(ControllerRole, design.role) if design.role in {"guidance", "attitude", "rate", "local_regulator", "integral_regulator", "allocator", "actuator", "fallback", "baseline", "unspecified"} else "unspecified",
        implementation="lqr",
        implementation_version=design.implementation_version,
        fidelity=cast(ControllerFidelity, design.fidelity) if design.fidelity in {"point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"} else "rigid_body_6dof",
        design_id=design.id,
        states=channels,
        inputs=inputs,
        plant_source=design.plant_source,
        linearization_source=design.linearization_source,
        operating_point={"trim": design.trim, **dict(trim.spec.operating_point)},
        state_scale_id=design.state_scale_id,
        control_scale_id=design.control_scale_id,
        q_id=design.q_id,
        r_id=design.r_id,
        a_sha256=result.a_sha256,
        b_sha256=result.b_sha256,
        q_sha256=result.q_sha256,
        r_sha256=result.r_sha256,
        k_sha256=result.k_sha256,
        closed_loop_poles=poles,
        closed_loop_max_real_pole=result.maximum_real_pole,
        allocator_id=design.allocator,
        control_path=("guidance", "reference_shaping", "lqr", "allocator", "actuator", "plant"),
        fallback_controller_id=design.fallback_controller_id,
        scenario_overrides_allowed=design.scenario_overrides_allowed,
        claim_status="design",
        evidence_tier=design.evidence_tier,
        control_realization_path=design.control_realization_path,
        provenance={
            "trim": design.trim,
            "notes": design.notes,
            "screen_only": str(design.screen_only).lower(),
        },
    )
    ####


def _build_lqi_realization(design: ControllerDesignSpec, trim: TrimResult, result: object) -> ControllerRealization:
    """Build LQI provenance while retaining the physical state contract."""

    from .runtime.lqr import LqiResult

    if not isinstance(result, LqiResult):
        raise TypeError("LQI realization requires an LqiResult")
    state_units = design.state_units or tuple("unspecified" for _ in design.states)
    state_frames = design.state_frames or tuple("unspecified" for _ in design.states)
    control_units = design.control_units or tuple("unspecified" for _ in design.controls)
    control_frames = design.control_frames or tuple("unspecified" for _ in design.controls)
    states = tuple(
        ControllerChannel(name=name, order=index, unit=state_units[index], frame=state_frames[index], scale=1.0)
        for index, name in enumerate(design.states)
    )
    inputs = tuple(
        ControllerChannel(name=name, order=index, unit=control_units[index], frame=control_frames[index], scale=1.0)
        for index, name in enumerate(design.controls)
    )
    poles = tuple(ClosedLoopPole(real=float(value.real), imaginary=float(value.imag)) for value in result.design.closed_loop_eigenvalues)
    return ControllerRealization(
        id=f"{design.id}:realization",
        role=cast(ControllerRole, design.role)
        if design.role in {"guidance", "attitude", "rate", "local_regulator", "integral_regulator", "allocator", "actuator", "fallback", "baseline", "unspecified"}
        else "unspecified",
        implementation="lqi",
        implementation_version=design.implementation_version,
        fidelity=cast(ControllerFidelity, design.fidelity)
        if design.fidelity in {"point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"}
        else "rigid_body_6dof",
        design_id=design.id,
        states=states,
        inputs=inputs,
        plant_source=design.plant_source,
        linearization_source=design.linearization_source,
        operating_point={"trim": design.trim, **dict(trim.spec.operating_point)},
        state_scale_id=design.state_scale_id,
        control_scale_id=design.control_scale_id,
        q_id=design.q_id,
        r_id=design.r_id,
        a_sha256=result.design.a_sha256,
        b_sha256=result.design.b_sha256,
        q_sha256=result.design.q_sha256,
        r_sha256=result.design.r_sha256,
        k_sha256=result.design.k_sha256,
        integral_states=design.integral_outputs,
        closed_loop_poles=poles,
        closed_loop_max_real_pole=result.maximum_real_pole,
        allocator_id=design.allocator,
        control_path=("guidance", "reference_shaping", "lqi", "allocator", "actuator", "plant"),
        fallback_controller_id=design.fallback_controller_id,
        scenario_overrides_allowed=design.scenario_overrides_allowed,
        claim_status="design",
        evidence_tier=design.evidence_tier,
        control_realization_path=design.control_realization_path,
        provenance={
            "trim": design.trim,
            "integral_outputs": ",".join(design.integral_outputs),
            "notes": design.notes,
            "screen_only": str(design.screen_only).lower(),
        },
    )
    ####
