"""Tests for the explicit X8 physical-surface route boundary packet."""

from __future__ import annotations

import json
from pathlib import Path

from tools.validate_x8_physical_surface_boundary import write_boundary_witness


def test_x8_surface_witness_passes_short_allocation_and_fails_closed_long_route(tmp_path: Path) -> None:
    """The packet must preserve both physical evidence and the beta boundary."""

    payload = write_boundary_witness(tmp_path)
    short = payload["short_witness"]
    boundary = payload["full_route_boundary"]
    assert short["status"] == "short_source_domain_witness_pass"
    assert short["mission_pass"] is True
    assert short["active_surface_counts"] == [2]
    assert short["effectiveness_rank"] == [2]
    assert short["direct_moment_active_samples"] == 0
    assert short["max_controlled_residual_nm"] < 1.0e-4
    assert short["max_abs_propulsion_yaw_moment_nm"] == 0.0
    assert boundary["status"] == "source_beta_boundary_fail_closed"
    assert boundary["mission_pass"] is False
    assert boundary["beta_boundary_reported"] is True
    assert boundary["envelope_exit_reported"] is True
    manifest = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["report"] == "boundary_witness.json"
    ####

