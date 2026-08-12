"""Small provider-neutral episodes for the low-fidelity control API ladder.

This module owns lifecycle plumbing only.  Providers supply the state
transition that already defines their model, so adding a session surface does
not add a second physics or guidance implementation.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path

from .composition_episode import (
    ActionFrame,
    EpisodeChannel,
    EpisodeObservation,
    EpisodeStatus,
    EpisodeStep,
    ObservationFrame,
    StatusFrame,
)
from .vehicle_interface import (
    InterfaceChannel,
    VehicleInterfaceContract,
    validate_authority_action_values,
)

FixtureState = Mapping[str, object]
InitialStateFactory = Callable[[int | None], FixtureState]
ObservationFactory = Callable[[FixtureState, float, EpisodeStatus], Mapping[str, object]]
TransitionFunction = Callable[[FixtureState, str, Mapping[str, object], float, float], "FixtureTransition"]
AuthoritySelectionHook = Callable[[FixtureState, str, str], FixtureState]


@dataclass(frozen=True, slots=True)
class FixtureTransition:
    """One provider-computed transition and its explicit lowering evidence."""

    state: FixtureState
    applied_action: Mapping[str, object]
    applied_semantic_action: Mapping[str, object] | None = None
    lowering_evidence: Mapping[str, object] = field(default_factory=dict)
    events: tuple[str, ...] = ()
    diagnostics: tuple[str, ...] = ()
    status: EpisodeStatus = "active"


class FixtureCompositionEpisode:
    """Reusable lifecycle adapter for an existing low-fidelity transition.

    ``auto_select_default_authority`` tells the provider-neutral session
    manager that omission of an authority ID means the advertised default,
    rather than the legacy native-union compatibility surface used by older
    canonical vehicle episodes.
    """

    auto_select_default_authority = True
    checkpoint_schema = "taoryx.fixture-composition-episode-checkpoint/v1"

    def __init__(
        self,
        *,
        interface_contract: VehicleInterfaceContract,
        observation_schema: tuple[EpisodeChannel, ...],
        initial_state_factory: InitialStateFactory,
        observation_factory: ObservationFactory,
        transition: TransitionFunction,
        claim_boundary: str,
        seed: int | None = None,
        authority_selection_hook: AuthoritySelectionHook | None = None,
    ) -> None:
        if not claim_boundary.strip():
            raise ValueError("fixture episode requires a claim boundary")
        if not observation_schema:
            raise ValueError("fixture episode requires at least one observation channel")
        self.interface_contract = interface_contract
        self.claim_boundary = claim_boundary
        self._observation_schema = observation_schema
        self._initial_state_factory = initial_state_factory
        self._observation_factory = observation_factory
        self._transition = transition
        self._authority_selection_hook = authority_selection_hook
        self._seed = seed
        self._state = dict(initial_state_factory(seed))
        self._time_s = 0.0
        self._status: EpisodeStatus = "ready"
        self._active_authority_profile_id = interface_contract.default_authority_profile_id
        self._last_lowering_evidence: dict[str, object] = {}
        ####

    @property
    def action_schema(self) -> tuple[EpisodeChannel, ...]:
        """Return the full semantic union for legacy inspection only."""

        return tuple(_episode_channel(item) for item in self.interface_contract.action_channels)
        ####

    @property
    def observation_schema(self) -> tuple[EpisodeChannel, ...]:
        return self._observation_schema
        ####

    @property
    def active_authority_profile_id(self) -> str | None:
        return self._active_authority_profile_id
        ####

    def reset(self, *, seed: int | None = None) -> EpisodeObservation:
        """Reconstruct the prepared initial state without changing authority."""

        if self._status == "closed":
            raise RuntimeError("cannot reset a closed fixture episode")
        if seed is not None:
            self._seed = seed
        self._state = dict(self._initial_state_factory(self._seed))
        self._time_s = 0.0
        self._status = "ready"
        self._last_lowering_evidence = {}
        return self.observe()
        ####

    def observe(self) -> EpisodeObservation:
        values = dict(self._observation_factory(self._state, self._time_s, self._status))
        return EpisodeObservation(self._time_s, values, self._status)
        ####

    def step(self, action: Mapping[str, object], duration_s: float) -> EpisodeStep:
        """Apply through the selected default profile, never an implicit union."""

        profile_id = self._active_authority_profile_id
        if profile_id is None:
            raise RuntimeError("fixture episode has no selected authority profile")
        return self.step_frame(
            ActionFrame(
                self.interface_contract.id,
                self.interface_contract.fingerprint,
                profile_id,
                action,
                duration_s,
            )
        )
        ####

    def step_frame(self, action: ActionFrame) -> EpisodeStep:
        if self._status == "closed":
            raise RuntimeError("cannot step a closed fixture episode")
        if action.interface_id != self.interface_contract.id:
            raise ValueError("action frame names another interface")
        if action.interface_fingerprint_sha256 != self.interface_contract.fingerprint:
            raise ValueError("action frame interface fingerprint is stale")
        if action.authority_profile_id != self._active_authority_profile_id:
            raise ValueError(
                f"action frame selects {action.authority_profile_id!r}; "
                f"active profile is {self._active_authority_profile_id!r}"
            )
        profile = validate_authority_action_values(
            self.interface_contract,
            action.authority_profile_id,
            action.values,
            context="fixture semantic action",
        )
        start_s = self._time_s
        result = self._transition(
            dict(self._state),
            profile.id,
            dict(action.values),
            action.duration_s,
            start_s,
        )
        self._state = dict(result.state)
        self._time_s = start_s + action.duration_s
        self._status = result.status
        self._last_lowering_evidence = dict(result.lowering_evidence)
        observation = self.observe()
        status_frame = self.status_frame()
        observation_frame = self.observe_frame()
        applied_semantic = dict(
            action.values
            if result.applied_semantic_action is None
            else result.applied_semantic_action
        )
        return EpisodeStep(
            start_s,
            self._time_s,
            dict(action.values),
            dict(result.applied_action),
            observation,
            tuple(result.events),
            tuple(result.diagnostics),
            action,
            applied_semantic,
            observation_frame,
            status_frame,
        )
        ####

    def select_authority_profile(self, authority_profile_id: str) -> None:
        """Transfer profile identity while preserving the committed plant state."""

        selected = self.interface_contract.authority_profile(authority_profile_id)
        if selected.availability != "available":
            raise ValueError(f"authority profile {selected.id!r} is {selected.availability}")
        previous = self._active_authority_profile_id
        if previous == selected.id:
            return
        if previous is not None and self._authority_selection_hook is not None:
            self._state = dict(self._authority_selection_hook(dict(self._state), previous, selected.id))
        self._active_authority_profile_id = selected.id
        self._last_lowering_evidence = {
            "transition": f"{previous}->{selected.id}",
            "state_continuous": True,
        }
        ####

    def status_frame(self) -> StatusFrame:
        observation = self.observe()
        values = dict(observation.values)
        raw = {
            "fixture_state": _json_safe(self._state),
            "active_authority_profile_id": self._active_authority_profile_id,
            "control_lowering": dict(self._last_lowering_evidence),
        }
        return StatusFrame(
            self.interface_contract.id,
            self.interface_contract.fingerprint,
            self._time_s,
            values,
            raw,
            self._status,
        )
        ####

    def observe_frame(self, observation_profile_id: str = "truth_debug") -> ObservationFrame:
        available = tuple(
            item for item in self.interface_contract.observation_profiles if item.availability == "available"
        )
        if not available:
            raise ValueError("fixture interface has no available observation profile")
        selected = next((item for item in available if item.id == observation_profile_id), available[0])
        observation = self.observe()
        values = {
            identifier: observation.values[identifier]
            for identifier in selected.channel_ids
            if identifier in observation.values
        }
        return ObservationFrame(
            self.interface_contract.id,
            self.interface_contract.fingerprint,
            selected.id,
            self._time_s,
            values,
            {identifier: True for identifier in values},
            self._status,
            self._time_s,
        )
        ####

    def save_checkpoint(self, path: str | Path) -> Path:
        """Persist portable fixture state with interface-fingerprint binding."""

        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema": self.checkpoint_schema,
            "interface_id": self.interface_contract.id,
            "interface_fingerprint_sha256": self.interface_contract.fingerprint,
            "time_s": self._time_s,
            "status": self._status,
            "seed": self._seed,
            "active_authority_profile_id": self._active_authority_profile_id,
            "state": _json_safe(self._state),
        }
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=target.parent,
            prefix=f".{target.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            temporary = Path(stream.name)
        os.replace(temporary, target)
        return target
        ####

    def load_checkpoint(self, path: str | Path) -> EpisodeObservation:
        if self._status == "closed":
            raise RuntimeError("cannot restore a closed fixture episode")
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(payload, dict) or payload.get("schema") != self.checkpoint_schema:
            raise ValueError("fixture checkpoint schema is unsupported")
        if payload.get("interface_id") != self.interface_contract.id:
            raise ValueError("fixture checkpoint names another interface")
        if payload.get("interface_fingerprint_sha256") != self.interface_contract.fingerprint:
            raise ValueError("fixture checkpoint interface fingerprint is stale")
        state = payload.get("state")
        if not isinstance(state, dict):
            raise ValueError("fixture checkpoint state must be an object")
        time_s = payload.get("time_s")
        if isinstance(time_s, bool) or not isinstance(time_s, int | float) or not math.isfinite(float(time_s)) or time_s < 0:
            raise ValueError("fixture checkpoint time_s must be finite and nonnegative")
        status = payload.get("status")
        if status not in {"ready", "active", "completed"}:
            raise ValueError("fixture checkpoint status is invalid")
        profile_id = payload.get("active_authority_profile_id")
        if profile_id is not None:
            if not isinstance(profile_id, str):
                raise ValueError("fixture checkpoint authority profile is invalid")
            self.interface_contract.authority_profile(profile_id)
        self._state = dict(state)
        self._time_s = float(time_s)
        self._status = status
        self._seed = payload.get("seed") if isinstance(payload.get("seed"), int) else None
        self._active_authority_profile_id = profile_id
        self._last_lowering_evidence = {"checkpoint_restored": True}
        return self.observe()
        ####

    def close(self) -> None:
        self._status = "closed"
        ####


def _episode_channel(channel: InterfaceChannel) -> EpisodeChannel:
    """Project a semantic action without erasing topology or units."""

    return EpisodeChannel(
        channel.id,
        channel.canonical_unit,
        channel.lower,
        channel.upper,
        channel.description,
        channel.value_space,
    )
    ####


def _json_safe(value: object) -> object:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, tuple | list):
        return [_json_safe(item) for item in value]
    return value
    ####


__all__ = ["FixtureCompositionEpisode", "FixtureTransition"]
