"""Source-replay adapters for the NASA/NESC two-stage rocket family.

The retained NESC package provides qualified source translation history and a
named pseudo attitude-response composition, but it does not expose a
participating state derivative or authoritative gimbal effectivity.  This
module integrates those products through the optional ``replay`` capability
instead of misrepresenting a history replay as a nonlinear plant.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path

from .family_adapter import AdapterChannel, FamilyAdapterDescriptor, StandardFamilyAdapter
from .fidelity_contracts import FidelityTier
from .trajectory.nesc_pseudo6dof import DEFAULT_REDUCTION, build_nesc_composite_pseudo6dof

NESC_REPLAY_STATE_CHANNELS: tuple[AdapterChannel, ...] = (
    AdapterChannel("position_eci_m", "m", "state", frame="ECI"),
    AdapterChannel("velocity_eci_mps", "m/s", "state", frame="ECI"),
    AdapterChannel("mass_kg", "kg", "resource"),
    AdapterChannel("phase", "1", "diagnostic"),
)
NESC_PSEUDO_STATE_CHANNELS: tuple[AdapterChannel, ...] = NESC_REPLAY_STATE_CHANNELS + (
    AdapterChannel("commanded_attitude_rad", "rad", "response_state", frame="body"),
    AdapterChannel("achieved_attitude_rad", "rad", "response_state", frame="body"),
    AdapterChannel("body_rate_rad_s", "rad/s", "response_state", frame="body"),
)


def _source_artifact(request: Mapping[str, object]) -> Path:
    """Resolve the pinned reduction artifact, allowing explicit test fixtures."""

    value = request.get("source_artifact", DEFAULT_REDUCTION)
    if not isinstance(value, (str, Path)):
        raise TypeError("NESC source_artifact must be a path or string")
    return Path(value)
    ####


def _point_replay(request: Mapping[str, object]) -> dict[str, object]:
    """Return retained source translation rows without inventing derivatives."""

    artifact = _source_artifact(request)
    payload = json.loads(artifact.read_text(encoding="utf-8"))
    history = payload.get("history")
    if not isinstance(history, list) or not history:
        raise ValueError("NESC source replay artifact has no history rows")
    rows = [row for row in history if isinstance(row, dict)]
    if len(rows) != len(history):
        raise ValueError("NESC source replay history contains a non-mapping row")
    return {
        "schema": "taoryx.nesc-source-replay/v1alpha1",
        "family_id": "reference_nesc_two_stage_rocket",
        "fidelity": "point_mass_3dof",
        "source_artifact": str(artifact),
        "claim_boundary": "source translation history replay; not a participating nonlinear plant or guidance controller",
        "rows": rows,
    }
    ####


def _pseudo_replay(request: Mapping[str, object]) -> dict[str, object]:
    """Return source translation paired with the named response-law surrogate."""

    artifact = _source_artifact(request)
    result = build_nesc_composite_pseudo6dof(artifact)
    payload = result.as_dict()
    payload["fidelity"] = "pseudo_6dof"
    payload["control_realization"] = "response_law"
    return payload
    ####


def build_nesc_replay_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Build the NESC replay adapter for its honest reduced tiers."""

    if tier not in ("point_mass_3dof", "pseudo_6dof"):
        raise ValueError(f"NESC source replay does not implement {tier!r}")
    descriptor = FamilyAdapterDescriptor(
        family_id="reference_nesc_two_stage_rocket",
        adapter_id="taoryx.rocket.variable_mass_nesc.v1",
        physical_family="mass_depleting_rocket",
        tier=tier,
        state_channels=NESC_PSEUDO_STATE_CHANNELS if tier == "pseudo_6dof" else NESC_REPLAY_STATE_CHANNELS,
        resource_channels=(AdapterChannel("mass_kg", "kg", "resource"),),
        evidence_status="development",
        validity_envelope="Pinned NASA/NESC Scenario 17 source history through the retained 200 s endpoint",
        omitted_physics=(
            "participating nonlinear derivative",
            "active guidance and control",
            "source-exact attitude comparison",
            "physical thrust-vector/gimbal allocation",
            "plume and structural interaction",
        ),
    )
    provider = _point_replay if tier == "point_mass_3dof" else _pseudo_replay
    return StandardFamilyAdapter.from_replay(descriptor, provider)
    ####


__all__ = [
    "NESC_PSEUDO_STATE_CHANNELS",
    "NESC_REPLAY_STATE_CHANNELS",
    "build_nesc_replay_adapter",
]
####
