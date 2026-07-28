"""Validated continuous-time LQR contracts for TAORYX extensions."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from importlib.util import find_spec
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ..controller_realization import ControllerRealization


class LqrUnavailableError(RuntimeError):
    """Raised when the optional numerical backend for LQR is unavailable."""
####


@dataclass(frozen=True, slots=True)
class LqrSpec:
    """Problem-file declaration resolved into a controller configuration."""

    name: str
    states: tuple[str, ...]
    controls: tuple[str, ...]
    q_source: str | None = None
    r_source: str | None = None
    linearization_source: str | None = None
    method: str = "continuous"
    update: str = "initial"
####


@dataclass(frozen=True, slots=True)
class LqrResult:
    """Solved LQR gain and diagnostics."""

    gain: Any
    closed_loop_eigenvalues: Any
    controllable: bool
    condition_number: float
    state_names: tuple[str, ...]
    control_names: tuple[str, ...]
    a_sha256: str | None = None
    b_sha256: str | None = None
    q_sha256: str | None = None
    r_sha256: str | None = None
    k_sha256: str | None = None

    @property
    def maximum_real_pole(self) -> float:
        """Return the largest real part among the closed-loop poles."""

        return max(float(value.real) for value in self.closed_loop_eigenvalues)

    @property
    def hurwitz(self) -> bool:
        """Whether every closed-loop pole is strictly in the left half-plane."""

        return self.maximum_real_pole < 0.0

    @property
    def unstable_poles(self) -> tuple[Any, ...]:
        """Return poles that are nonnegative in real part."""

        return tuple(value for value in self.closed_loop_eigenvalues if float(value.real) >= 0.0)
    ####
####


@dataclass(frozen=True, slots=True)
class LqrCommand:
    """Named bounded control command produced by an LQR controller."""

    controls: Mapping[str, float]
    unsaturated: Mapping[str, float]
    saturated: tuple[str, ...]
####


@dataclass(frozen=True, slots=True)
class LqrUncertaintySpec:
    """Bounded fractional uncertainty used for controller screening.

    The envelope applies to the supplied local ``A`` and ``B`` derivatives;
    it is a robustness screen, not a statistical confidence interval.  Aero
    tables and derivatives are estimates, so callers should use a conservative
    engineering bound and retain the source/provenance of that bound.
    """

    a_fraction: float = 0.0
    b_fraction: float = 0.0
    samples: int = 9
    seed: int = 1995

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) and 0.0 <= value < 1.0 for value in (self.a_fraction, self.b_fraction)):
            raise ValueError("LQR uncertainty fractions must be finite and in [0, 1)")
        if self.samples < 1:
            raise ValueError("LQR uncertainty sample count must be positive")
        ####
    ####


@dataclass(frozen=True, slots=True)
class LqrRobustnessReport:
    """Closed-loop pole screen across a declared derivative envelope."""

    nominal_max_real_pole: float
    worst_max_real_pole: float
    samples: int
    stable: bool

    @property
    def margin_to_instability(self) -> float:
        """Return the pole margin, positive when the sampled envelope is stable."""

        return -self.worst_max_real_pole
    ####
####


@dataclass(frozen=True, slots=True)
class LqrController:
    """Apply a solved gain to named runtime state values."""

    result: LqrResult
    state_trim: Mapping[str, float] = field(default_factory=dict)
    control_trim: Mapping[str, float] = field(default_factory=dict)
    lower: Mapping[str, float] = field(default_factory=dict)
    upper: Mapping[str, float] = field(default_factory=dict)
    robustness: LqrRobustnessReport | None = None
    realization: ControllerRealization | None = None
    state_adapter: Callable[[Mapping[str, float]], Mapping[str, float]] | None = None

    def command(self, state: Mapping[str, float]) -> LqrCommand:
        """Return ``u_trim - K(x - x_trim)`` with optional saturation."""

        import numpy as np

        if self.state_adapter is not None:
            state = self.state_adapter(state)
        missing = [name for name in self.result.state_names if name not in state]
        if missing:
            raise KeyError(f"LQR state is missing: {', '.join(missing)}")
        state_error = np.asarray([float(state[name]) - float(self.state_trim.get(name, 0.0)) for name in self.result.state_names])
        raw = np.asarray(self.result.control_names and self.result.gain @ state_error).reshape(-1)
        unsaturated = {
            name: float(self.control_trim.get(name, 0.0) - value)
            for name, value in zip(self.result.control_names, raw, strict=True)
        }
        applied = {
            name: min(float(self.upper[name]), max(float(self.lower[name]), value))
            if name in self.lower and name in self.upper else value
            for name, value in unsaturated.items()
        }
        saturated = tuple(name for name in applied if applied[name] != unsaturated[name])
        return LqrCommand(applied, unsaturated, saturated)
    ####
####


LqrBuilder = Callable[[float | None, tuple[float, float, float] | None], LqrController]


@dataclass(slots=True)
class GainScheduledLqrController:
    """Cache LQR designs as mass or inertia operating conditions change.

    A schedule builder owns the plant physics.  The controller never infers
    inertia from mass: a vehicle may provide a source-backed inertia model,
    an explicit mass-property schedule, or keep inertia constant.  This is
    important for propellant vehicles where total mass changes but inertia
    does not necessarily scale linearly.
    """

    nominal: LqrController
    builder: LqrBuilder | None = None
    mass_tolerance_kg: float = 1.0e-6
    inertia_tolerance_fraction: float = 1.0e-6
    _active: LqrController = field(init=False)
    _last_mass_kg: float | None = field(default=None, init=False)
    _last_inertia: tuple[float, float, float] | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if not math.isfinite(self.mass_tolerance_kg) or self.mass_tolerance_kg <= 0.0:
            raise ValueError("mass scheduling tolerance must be finite and positive")
        if not math.isfinite(self.inertia_tolerance_fraction) or self.inertia_tolerance_fraction <= 0.0:
            raise ValueError("inertia scheduling tolerance must be finite and positive")
        self._active = self.nominal
        ####

    @property
    def result(self) -> LqrResult:
        """Expose the active result for telemetry and diagnostics."""

        return self._active.result
    ####

    @property
    def schedule_updates(self) -> int:
        """Return the number of operating-point designs selected so far."""

        return self._updates
    ####

    _updates: int = field(default=0, init=False)

    def command(
        self,
        state: Mapping[str, float],
        *,
        mass_kg: float | None = None,
        inertia: tuple[float, float, float] | None = None,
    ) -> LqrCommand:
        """Return a command using the cached design for the current plant point."""

        if mass_kg is not None and (not math.isfinite(mass_kg) or mass_kg <= 0.0):
            raise ValueError("scheduled LQR mass must be finite and positive")
        if inertia is not None:
            if len(inertia) != 3 or not all(math.isfinite(value) and value > 0.0 for value in inertia):
                raise ValueError("scheduled LQR inertia must contain three positive finite values")
            inertia = (float(inertia[0]), float(inertia[1]), float(inertia[2]))
        if self.builder is not None and self._needs_update(mass_kg, inertia):
            self._active = self.builder(mass_kg, inertia)
            self._last_mass_kg = mass_kg
            self._last_inertia = inertia
            self._updates += 1
        return self._active.command(state)
    ####

    def _needs_update(self, mass_kg: float | None, inertia: tuple[float, float, float] | None) -> bool:
        """Return whether the supplied operating point leaves the cached band."""

        if self._last_mass_kg is None and mass_kg is not None:
            return True
        if mass_kg is not None and self._last_mass_kg is not None and abs(mass_kg - self._last_mass_kg) > self.mass_tolerance_kg:
            return True
        if inertia is None:
            return False
        if self._last_inertia is None:
            return True
        return any(
            abs(current - previous) / max(abs(previous), 1.0e-12) > self.inertia_tolerance_fraction
            for current, previous in zip(inertia, self._last_inertia, strict=True)
        )
    ####
####


def solve_continuous_lqr(
    a: Sequence[Sequence[float]],
    b: Sequence[Sequence[float]],
    q: Sequence[Sequence[float]],
    r: Sequence[Sequence[float]],
    *,
    state_names: Sequence[str] = (),
    control_names: Sequence[str] = (),
) -> LqrResult:
    """Solve ``u = -Kx`` for a continuous-time linear system.

    The solver deliberately accepts matrices from Python/runtime adapters while
    ``LqrSpec`` carries the problem-file declaration. This keeps matrix parsing,
    units, and model linearization separate from the numerical backend.
    """

    if find_spec("numpy") is None or find_spec("scipy") is None:
        raise LqrUnavailableError("continuous LQR requires the optional numpy and scipy dependencies")
    import numpy as np
    from scipy.linalg import solve_continuous_are

    matrices = tuple(np.asarray(item, dtype=float) for item in (a, b, q, r))
    a_matrix, b_matrix, q_matrix, r_matrix = matrices
    if a_matrix.ndim != 2 or b_matrix.ndim != 2 or q_matrix.ndim != 2 or r_matrix.ndim != 2:
        raise ValueError("LQR matrices must all be two-dimensional")
    n = a_matrix.shape[0]
    m = b_matrix.shape[1]
    if a_matrix.shape != (n, n):
        raise ValueError("LQR matrix A must be square")
    if b_matrix.shape[0] != n:
        raise ValueError("LQR matrix B must have the same row count as A")
    if q_matrix.shape != (n, n):
        raise ValueError("LQR matrix Q must have the same square shape as A")
    if r_matrix.shape != (m, m):
        raise ValueError("LQR matrix R must be square with one row per control")
    if not all(np.isfinite(item).all() for item in matrices):
        raise ValueError("LQR matrices must contain only finite values")
    if not np.allclose(q_matrix, q_matrix.T, atol=1e-10) or not np.allclose(r_matrix, r_matrix.T, atol=1e-10):
        raise ValueError("LQR matrices Q and R must be symmetric")
    if np.linalg.eigvalsh(q_matrix).min() < -1e-10:
        raise ValueError("LQR matrix Q must be positive semidefinite")
    if np.linalg.eigvalsh(r_matrix).min() <= 0.0:
        raise ValueError("LQR matrix R must be positive definite")
    controllability = np.hstack(tuple(np.linalg.matrix_power(a_matrix, power) @ b_matrix for power in range(n)))
    rank = int(np.linalg.matrix_rank(controllability))
    if rank != n:
        raise ValueError(f"LQR system is not controllable: rank {rank}, required {n}")
    solution = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
    gain = np.linalg.solve(r_matrix, b_matrix.T @ solution)
    closed_loop = np.linalg.eigvals(a_matrix - b_matrix @ gain)
    return LqrResult(
        gain=gain,
        closed_loop_eigenvalues=closed_loop,
        controllable=True,
        condition_number=float(np.linalg.cond(r_matrix)),
        state_names=tuple(state_names),
        control_names=tuple(control_names),
        a_sha256=_matrix_sha256(a_matrix),
        b_sha256=_matrix_sha256(b_matrix),
        q_sha256=_matrix_sha256(q_matrix),
        r_sha256=_matrix_sha256(r_matrix),
        k_sha256=_matrix_sha256(gain),
    )
####


def solve_scaled_continuous_lqr(
    a: Sequence[Sequence[float]],
    b: Sequence[Sequence[float]],
    q: Sequence[Sequence[float]],
    r: Sequence[Sequence[float]],
    *,
    state_scales: Sequence[float],
    control_scales: Sequence[float],
    state_names: Sequence[str] = (),
    control_names: Sequence[str] = (),
) -> LqrResult:
    """Solve LQR in dimensionless state and control coordinates.

    ``state_scales`` and ``control_scales`` define one-unit engineering
    magnitudes. The returned gain is mapped back to the physical state and
    control units, so ``LqrController`` remains unchanged at the call site.
    """

    import numpy as np

    state_scale = np.asarray(tuple(state_scales), dtype=float)
    control_scale = np.asarray(tuple(control_scales), dtype=float)
    if state_scale.ndim != 1 or control_scale.ndim != 1 or not np.isfinite(state_scale).all() or not np.isfinite(control_scale).all():
        raise ValueError("LQR state and control scales must be finite one-dimensional values")
    if np.any(state_scale <= 0.0) or np.any(control_scale <= 0.0):
        raise ValueError("LQR state and control scales must be strictly positive")
    if len(state_scale) != len(a) or len(control_scale) != len(b[0]):
        raise ValueError("LQR scale counts must match the state and control dimensions")
    state_to_normalized = np.diag(1.0 / state_scale)
    normalized_to_state = np.diag(state_scale)
    normalized_to_control = np.diag(control_scale)
    normalized_result = solve_continuous_lqr(
        state_to_normalized @ np.asarray(a, dtype=float) @ normalized_to_state,
        state_to_normalized @ np.asarray(b, dtype=float) @ normalized_to_control,
        q,
        r,
        state_names=state_names,
        control_names=control_names,
    )
    return replace(
        normalized_result,
        gain=normalized_to_control @ normalized_result.gain @ state_to_normalized,
        a_sha256=_matrix_sha256(np.asarray(a, dtype=float)),
        b_sha256=_matrix_sha256(np.asarray(b, dtype=float)),
        q_sha256=_matrix_sha256(np.asarray(q, dtype=float)),
        r_sha256=_matrix_sha256(np.asarray(r, dtype=float)),
        k_sha256=_matrix_sha256(normalized_to_control @ normalized_result.gain @ state_to_normalized),
    )
####


def assess_lqr_robustness(
    a: Sequence[Sequence[float]],
    b: Sequence[Sequence[float]],
    result: LqrResult,
    uncertainty: LqrUncertaintySpec,
) -> LqrRobustnessReport:
    """Screen a solved gain against bounded local ``A``/``B`` perturbations.

    The nominal gain is held fixed while the local derivatives vary.  This is
    intentionally a fail-closed diagnostic for estimated aerodynamic models;
    it does not claim formal robust stability for the continuous uncertainty
    set between samples.
    """

    import numpy as np

    a_matrix = np.asarray(a, dtype=float)
    b_matrix = np.asarray(b, dtype=float)
    gain = np.asarray(result.gain, dtype=float)
    if a_matrix.ndim != 2 or b_matrix.ndim != 2 or gain.shape != (b_matrix.shape[1], a_matrix.shape[0]):
        raise ValueError("LQR robustness matrices and gain have incompatible dimensions")
    rng = np.random.default_rng(uncertainty.seed)
    perturbations: list[tuple[np.ndarray, np.ndarray]] = [(np.zeros_like(a_matrix), np.zeros_like(b_matrix))]
    if uncertainty.samples > 1 and (uncertainty.a_fraction > 0.0 or uncertainty.b_fraction > 0.0):
        for _ in range(uncertainty.samples - 1):
            delta_a = rng.uniform(-uncertainty.a_fraction, uncertainty.a_fraction, size=a_matrix.shape) * np.abs(a_matrix)
            delta_b = rng.uniform(-uncertainty.b_fraction, uncertainty.b_fraction, size=b_matrix.shape) * np.abs(b_matrix)
            perturbations.append((delta_a, delta_b))
    poles: list[Any] = []
    for delta_a, delta_b in perturbations:
        poles.extend(np.linalg.eigvals((a_matrix + delta_a) - (b_matrix + delta_b) @ gain))
    real_parts = [float(value.real) for value in poles]
    return LqrRobustnessReport(
        nominal_max_real_pole=float(max(float(value.real) for value in result.closed_loop_eigenvalues)),
        worst_max_real_pole=max(real_parts),
        samples=len(perturbations),
        stable=max(real_parts) < 0.0,
    )
####


def _matrix_sha256(matrix: Any) -> str:
    """Hash a finite numeric matrix with shape and dtype in the payload."""

    import numpy as np

    array = np.ascontiguousarray(np.asarray(matrix, dtype=np.float64))
    header = json.dumps({"shape": list(array.shape), "dtype": str(array.dtype)}, sort_keys=True).encode("utf-8")
    return hashlib.sha256(header + b"\0" + array.tobytes(order="C")).hexdigest()
####
