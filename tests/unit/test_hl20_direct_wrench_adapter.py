"""Tests for the explicit HL-20 source-load direct-wrench bridge."""

from __future__ import annotations

import math

from taoryx.direct_wrench import DIRECT_WRENCH_NAMES
from taoryx.hl20_adapter import (
    build_hl20_local_direct_wrench_screen_config,
    build_hl20_mach2_authority_probe_config,
    build_hl20_source_direct_wrench_adapter,
)
from taoryx.hl20_reachability import HL20_FIXED_MASS_KG
from taoryx.local_direct_wrench import run_local_direct_wrench_screen


def _state() -> dict[str, float]:
    speed = 340.294
    alpha_rad = math.radians(5.0)
    return {
        "u_m_s": speed * math.cos(alpha_rad),
        "v_m_s": 0.0,
        "w_m_s": -speed * math.sin(alpha_rad),
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
        "altitude_m": 0.0,
    }


def test_hl20_direct_wrench_bridge_has_no_surface_claim() -> None:
    adapter = build_hl20_source_direct_wrench_adapter()
    descriptor = adapter.describe()

    assert descriptor.tier == "rigid_body_6dof_direct_wrench"
    assert descriptor.control_realization == "direct_wrench"
    assert all(channel.role == "direct_wrench" for channel in descriptor.control_channels)
    assert adapter.capability_report().capability("allocate").status == "not_applicable"
    assert adapter.capability_report().capability("trim").status == "not_applicable"
    assert adapter.capability_report().capability("trim_fragment").status == "available"


def test_hl20_direct_wrench_bridge_exposes_only_scalar_trim_fragment() -> None:
    adapter = build_hl20_source_direct_wrench_adapter()

    fragment = adapter.trim_fragment({"mach": 1.0})

    assert fragment.status == "verified"
    assert fragment.fragment_id == "hl20_source_pitch_channel_trim"
    assert set(fragment.state) == {"alpha_deg"}
    assert fragment.controls == {}
    assert "not full 6-DOF equilibrium" in fragment.claim_boundary


def test_hl20_direct_wrench_changes_local_acceleration() -> None:
    adapter = build_hl20_source_direct_wrench_adapter()
    state = _state()
    zero = {name: 0.0 for name in adapter.control_names}
    requested = dict(zero)
    requested["force_x_n"] = 10_000.0

    baseline = adapter.state_derivative(state, zero, {})
    achieved = adapter.state_derivative(state, requested, {})

    assert achieved["u_m_s"] > baseline["u_m_s"]
    assert math.isclose(
        achieved["u_m_s"] - baseline["u_m_s"],
        10_000.0 / HL20_FIXED_MASS_KG,
        rel_tol=1.0e-12,
    )


def test_hl20_local_direct_wrench_config_reveals_insufficient_declared_force_authority() -> None:
    """Do not silently widen the bridge merely to call the local screen trimmed."""

    config = build_hl20_mach2_authority_probe_config()
    bias = config.balancing_wrench(config.reference_state)

    assert any(
        float(bias[name]) < float(config.limits.lower[name])
        or float(bias[name]) > float(config.limits.upper[name])
        for name in DIRECT_WRENCH_NAMES
    )


def test_hl20_local_direct_wrench_screen_fails_at_the_equilibrium_gate() -> None:
    """A non-equilibrium local screen must not be presented as an LQR success."""

    screen = run_local_direct_wrench_screen(build_hl20_mach2_authority_probe_config())

    assert screen.equilibrium_pass is False
    assert screen.equilibrium_projection.status == "partially_achievable"
    assert screen.equilibrium_derivative_norm > screen.config.equilibrium_derivative_norm_limit
    assert screen.mission_pass is False
    assert screen.as_dict()["evaluation"]["equilibrium"]["passed"] is False


def test_hl20_source_feasible_local_direct_wrench_screen_passes_without_widening_limits() -> None:
    """The public local screen uses a source-domain point the bridge can hold."""

    screen = run_local_direct_wrench_screen(build_hl20_local_direct_wrench_screen_config())

    assert screen.config.id == "hl20-source-mach0p5-local-direct-wrench-v1"
    assert screen.equilibrium_pass is True
    assert screen.equilibrium_projection.status == "feasible"
    assert screen.observed_statuses == ("feasible",)
    assert screen.mission_pass is True
