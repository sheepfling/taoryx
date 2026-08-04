"""Generic policy execution through declared composition interfaces.

The runtime intentionally gives a policy neither a native state-vector layout
nor a vehicle-specific control dictionary.  A policy receives only one
declared observation profile and submits one declared authority-profile frame.
The episode remains responsible for mapping that semantic request to the
native implementation and for reporting the actual applied command.

This is an execution harness, not an optimizer or reinforcement-learning
framework.  It establishes the stable contract that those tools will use
without inventing observations, effectors, or a second dynamics path.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import cast

from .composition_episode import (
    ActionFrame,
    EpisodeStep,
    ObservationFrame,
    StatusFrame,
    VehicleCompositionEpisode,
    open_vehicle_composition_episode,
)
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_interface import VehicleInterfaceContract, project_authority_action_values


@dataclass(frozen=True, slots=True)
class PolicyDecision:
    """One semantic action chosen from a declared observation boundary."""

    values: Mapping[str, object]
    duration_s: float

    def as_dict(self) -> dict[str, object]:
        """Return a replayable policy decision without native translation."""

        return {"values": dict(self.values), "duration_s": self.duration_s}
        ####
    ####


PolicyFunction = Callable[[ObservationFrame, VehicleInterfaceContract], PolicyDecision | None]


@dataclass(frozen=True, slots=True)
class CompositionPolicyTrace:
    """Audit record for one profile-bound policy episode execution."""

    composition_id: str
    composition_identity_sha256: str
    interface_id: str
    interface_fingerprint_sha256: str
    authority_profile_id: str
    observation_profile_id: str
    initial_observation: ObservationFrame
    steps: tuple[EpisodeStep, ...]
    final_observation: ObservationFrame
    final_status: StatusFrame
    stopped_by_policy: bool
    integration_step_s: float | None = None

    def as_dict(self) -> dict[str, object]:
        """Return the complete semantic action and committed-truth trace."""

        return {
            "schema": "taoryx.composition-policy-trace/v1alpha1",
            "composition_id": self.composition_id,
            "composition_identity_sha256": self.composition_identity_sha256,
            "interface_id": self.interface_id,
            "interface_fingerprint_sha256": self.interface_fingerprint_sha256,
            "authority_profile_id": self.authority_profile_id,
            "observation_profile_id": self.observation_profile_id,
            "initial_observation": self.initial_observation.as_dict(),
            "steps": [step.as_dict() for step in self.steps],
            "final_observation": self.final_observation.as_dict(),
            "final_status": self.final_status.as_dict(),
            "stopped_by_policy": self.stopped_by_policy,
            "integration_step_s": self.integration_step_s,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class CompositionPolicyReplayReport:
    """Exact scripted replay verdict for one semantic policy trace."""

    composition_id: str
    composition_identity_sha256: str
    interface_id: str
    interface_fingerprint_sha256: str
    step_count: int
    final_time_s: float

    def as_dict(self) -> dict[str, object]:
        """Return a compact machine-readable deterministic replay verdict."""

        return {
            "schema": "taoryx.composition-policy-replay/v1alpha1",
            "status": "pass",
            "composition_id": self.composition_id,
            "composition_identity_sha256": self.composition_identity_sha256,
            "interface_id": self.interface_id,
            "interface_fingerprint_sha256": self.interface_fingerprint_sha256,
            "step_count": self.step_count,
            "final_time_s": self.final_time_s,
            "claim_boundary": (
                "This verifies deterministic replay of one declared semantic action stream through the same "
                "episode kernel. It does not establish batch parity, robustness, physical-effector realization, "
                "or mission qualification."
            ),
        }
        ####
    ####


def run_composition_policy(
    episode: VehicleCompositionEpisode,
    policy: PolicyFunction,
    *,
    authority_profile_id: str,
    observation_profile_id: str | None = None,
    maximum_decisions: int = 1000,
) -> CompositionPolicyTrace:
    """Run a policy only through one declared semantic profile pair.

    Returning ``None`` from the policy ends the trace at the current truth
    boundary.  Hitting ``maximum_decisions`` is an error: a caller must choose
    an explicit finite horizon rather than accepting a silent partial run.
    """

    if maximum_decisions <= 0:
        raise ValueError("maximum_decisions must be positive")
    contract = episode.interface_contract
    resolved_observation_profile_id = observation_profile_id or episode.composition.observation.profile_id
    authority = contract.authority_profile(authority_profile_id)
    observation_profile = contract.observation_profile(resolved_observation_profile_id)
    if authority.availability != "available":
        raise ValueError(f"authority profile {authority.id!r} is not executable")
    if observation_profile.availability != "available":
        raise ValueError(f"observation profile {observation_profile.id!r} is not executable")

    initial = episode.observe_frame(resolved_observation_profile_id)
    observation = initial
    steps: list[EpisodeStep] = []
    for _ in range(maximum_decisions):
        decision = policy(observation, contract)
        if decision is None:
            return CompositionPolicyTrace(
                episode.composition.id,
                episode.composition.identity_sha256,
                contract.id,
                contract.fingerprint,
                authority_profile_id,
                resolved_observation_profile_id,
                initial,
                tuple(steps),
                observation,
                episode.status_frame(),
                True,
                _episode_integration_step(episode),
            )
        projected_values = project_authority_action_values(
            contract,
            authority_profile_id,
            decision.values,
            context=f"policy decision for authority profile {authority_profile_id!r}",
        )
        frame = ActionFrame(
            contract.id,
            contract.fingerprint,
            authority_profile_id,
            projected_values,
            decision.duration_s,
        )
        step = episode.step_frame(frame)
        steps.append(step)
        observation = episode.observe_frame(resolved_observation_profile_id)
    raise RuntimeError(f"composition policy exceeded maximum_decisions={maximum_decisions}")
    ####


def replay_composition_policy_trace(
    composition: CompiledVehicleComposition,
    trace: CompositionPolicyTrace,
    *,
    seed: int | None = None,
) -> CompositionPolicyReplayReport:
    """Replay a previously emitted semantic trace through a fresh episode.

    This check deliberately reuses the public semantic frames rather than the
    native action sidecar.  It therefore catches interface drift, changed
    action mappings, altered accepted-boundary semantics, and nondeterministic
    episode state transitions without defining a second physics or policy API.
    """

    return replay_serialized_composition_policy_trace(composition, trace.as_dict(), seed=seed)
    ####


def write_composition_policy_trace(trace: CompositionPolicyTrace, path: str | Path) -> Path:
    """Write one exact semantic action trace for later independent replay."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(trace.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return destination
    ####


def replay_composition_policy_trace_file(
    composition: CompiledVehicleComposition,
    path: str | Path,
    *,
    seed: int | None = None,
) -> CompositionPolicyReplayReport:
    """Load and independently replay a persisted semantic action trace."""

    source = Path(path)
    payload = _mapping(json.loads(source.read_text(encoding="utf-8")), "policy trace")
    return replay_serialized_composition_policy_trace(composition, payload, seed=seed)
    ####


def replay_serialized_composition_policy_trace(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
    *,
    seed: int | None = None,
) -> CompositionPolicyReplayReport:
    """Replay one serialized semantic trace without trusting its producer.

    The persisted payload is compared against a fresh episode at every public
    boundary.  Parsing only the input action frame is deliberate: all other
    persisted records remain untrusted expected evidence and are compared to
    newly produced public records rather than reconstructed as native objects.
    """

    if _text(payload, "schema") != "taoryx.composition-policy-trace/v1alpha1":
        raise ValueError("unsupported policy trace schema")
    trace_composition_id = _text(payload, "composition_id")
    trace_identity = _text(payload, "composition_identity_sha256")
    if trace_composition_id != composition.id or trace_identity != composition.identity_sha256:
        raise ValueError("policy trace composition identity disagrees with the requested replay composition")
    episode = open_vehicle_composition_episode(
        composition,
        seed=seed,
        integration_step_s=_serialized_integration_step(payload),
    )
    try:
        contract = episode.interface_contract
        if _text(payload, "interface_id") != contract.id or _text(payload, "interface_fingerprint_sha256") != contract.fingerprint:
            raise ValueError("policy trace interface identity disagrees with the replay episode")
        observation_profile_id = _text(payload, "observation_profile_id")
        _require_equal_payload(
            "initial observation",
            episode.observe_frame(observation_profile_id).as_dict(),
            _mapping(payload.get("initial_observation"), "initial_observation"),
        )
        steps = _sequence(payload.get("steps"), "steps")
        for index, expected_value in enumerate(steps):
            expected = _mapping(expected_value, f"steps[{index}]")
            frame_payload = expected.get("action_frame")
            if frame_payload is None:
                raise ValueError(f"policy trace step {index} has no replayable semantic action frame")
            action_frame = _action_frame(_mapping(frame_payload, f"steps[{index}].action_frame"))
            actual = episode.step_frame(action_frame)
            _require_equal_payload(f"step {index}", actual.as_dict(), expected)
        _require_equal_payload(
            "final observation",
            episode.observe_frame(observation_profile_id).as_dict(),
            _mapping(payload.get("final_observation"), "final_observation"),
        )
        final_status = episode.status_frame()
        _require_equal_payload("final status", final_status.as_dict(), _mapping(payload.get("final_status"), "final_status"))
        return CompositionPolicyReplayReport(
            composition.id,
            composition.identity_sha256,
            contract.id,
            contract.fingerprint,
            len(steps),
            final_status.time_s,
        )
    finally:
        episode.close()
    ####


def _require_equal(label: str, actual: object, expected: object) -> None:
    """Compare serialized public records to avoid native-object equality leaks."""

    actual_payload = actual.as_dict() if hasattr(actual, "as_dict") else actual
    expected_payload = expected.as_dict() if hasattr(expected, "as_dict") else expected
    if actual_payload != expected_payload:
        raise ValueError(f"policy trace replay mismatch at {label}")
    ####


def _require_equal_payload(label: str, actual: Mapping[str, object], expected: Mapping[str, object]) -> None:
    """Compare fresh public evidence to untrusted serialized evidence."""

    if dict(actual) != dict(expected):
        raise ValueError(f"policy trace replay mismatch at {label}")
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    """Return one JSON object or reject malformed persisted evidence."""

    if not isinstance(value, Mapping):
        raise ValueError(f"policy trace {label} must be an object")
    return cast(Mapping[str, object], value)
    ####


def _sequence(value: object, label: str) -> tuple[object, ...]:
    """Return one JSON array or reject malformed persisted evidence."""

    if not isinstance(value, list):
        raise ValueError(f"policy trace {label} must be an array")
    return tuple(value)
    ####


def _text(payload: Mapping[str, object], key: str) -> str:
    """Read one required nonempty text field from persisted evidence."""

    value = payload.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"policy trace {key} must be nonempty text")
    return value
    ####


def _episode_integration_step(episode: VehicleCompositionEpisode) -> float | None:
    """Capture an explicitly declared native substep when a witness owns one."""

    value = getattr(episode, "integration_step_s", None)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int | float) or value <= 0.0:
        raise ValueError("episode integration_step_s must be a positive numeric value when exposed")
    return float(value)
    ####


def _serialized_integration_step(payload: Mapping[str, object]) -> float:
    """Read an optional persisted substep without imposing one on legacy traces."""

    value = payload.get("integration_step_s")
    if value is None:
        return 0.02
    if isinstance(value, bool) or not isinstance(value, int | float) or value <= 0.0:
        raise ValueError("policy trace integration_step_s must be positive numeric or null")
    return float(value)
    ####


def _action_frame(payload: Mapping[str, object]) -> ActionFrame:
    """Reconstruct only the externally submitted semantic action frame."""

    duration = payload.get("duration_s")
    if not isinstance(duration, (int, float)) or isinstance(duration, bool):
        raise ValueError("policy trace action frame duration_s must be numeric")
    return ActionFrame(
        _text(payload, "interface_id"),
        _text(payload, "interface_fingerprint_sha256"),
        _text(payload, "authority_profile_id"),
        _mapping(payload.get("values"), "action_frame.values"),
        float(duration),
    )
    ####


__all__ = [
    "CompositionPolicyTrace",
    "CompositionPolicyReplayReport",
    "PolicyDecision",
    "PolicyFunction",
    "replay_composition_policy_trace",
    "replay_composition_policy_trace_file",
    "replay_serialized_composition_policy_trace",
    "run_composition_policy",
    "write_composition_policy_trace",
]
