"""Validated continuous-time LQR contracts for TAORYX extensions."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from importlib.util import find_spec
from typing import Any


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
    )
####
