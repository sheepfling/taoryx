"""Portable, fail-closed application contexts for tuned controller candidates.

The tuning campaign produces a reproducible candidate, while a vehicle batch
factory owns the actual controller construction and plant-specific trim,
allocation, and limits.  This module is the narrow bridge between those two
responsibilities.  It carries one selected candidate's resolved gains and
coordinate contract, and it can emit a runtime binding only after a plug-in
has checked that its controller uses that exact contract.

It deliberately does not instantiate a controller on a plug-in's behalf:
doing so would hide family-specific trim, actuator, and realization choices.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from copy import deepcopy
from dataclasses import dataclass
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

TuningControllerMethod = Literal["lqr", "lqi"]


class RuntimeTuningBindingReceipt(BaseModel):
    """Typed receipt emitted only after a runtime applies one tuned candidate.

    The receipt is intentionally small enough to live inside a provider-owned
    ``runtime`` extension, while retaining every identity needed to bind that
    runtime to a cached campaign candidate.  It replaces the previous
    informal dictionary convention at the plug-in-to-result-catalog seam.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    campaign_id: str = Field(min_length=1)
    node_id: str = Field(min_length=1)
    candidate_profile_id: str = Field(min_length=1)
    candidate_configuration_fingerprint_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    applied_gain_fingerprint_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    controller_method: TuningControllerMethod
    cache_key: str | None = Field(default=None, min_length=1)

    def require_runtime_method(self, controller_method: object) -> None:
        """Reject a receipt that disagrees with the enclosing runtime method."""

        if self.controller_method != controller_method:
            raise ValueError("execution tuning_binding controller method disagrees with execution runtime")
        ####

    def as_dict(self) -> dict[str, str]:
        """Return the JSON-safe receipt for a provider runtime extension."""

        return self.model_dump(mode="json", exclude_none=True)
        ####

    ####


@dataclass(frozen=True, slots=True)
class TuningApplicationContext:
    """One selected campaign candidate ready for compatible runtime use.

    A context is intentionally more than a method label.  It retains the
    exact node/profile/configuration fingerprint, named coordinates, scales,
    weights, and resolved physical-coordinate gain matrices.  A plug-in must
    call :meth:`runtime_binding_after_application` only after it has actually
    instantiated and applied a controller compatible with those fields.
    """

    campaign_id: str
    node_id: str
    candidate_profile_id: str
    candidate_configuration_fingerprint_sha256: str
    candidate_payload_fingerprint_sha256: str
    controller_method: TuningControllerMethod
    state_names: tuple[str, ...]
    control_names: tuple[str, ...]
    integral_output_names: tuple[str, ...]
    state_scales: tuple[float, ...]
    control_scales: tuple[float, ...]
    weights: Mapping[str, tuple[float, ...]]
    resolved_gains: Mapping[str, tuple[tuple[float, ...], ...]]
    candidate_configuration: Mapping[str, object]
    cache_key: str
    cache_hit: bool
    cache_persisted: bool

    def __post_init__(self) -> None:
        for label, value in (
            ("campaign ID", self.campaign_id),
            ("node ID", self.node_id),
            ("candidate profile ID", self.candidate_profile_id),
            ("cache key", self.cache_key),
        ):
            if not value.strip():
                raise ValueError(f"tuning application context requires a non-empty {label}")
        for label, value in (
            ("candidate configuration", self.candidate_configuration_fingerprint_sha256),
            ("candidate payload", self.candidate_payload_fingerprint_sha256),
        ):
            _validate_sha256(value, label)
        if canonical_json_fingerprint(self.candidate_configuration) != self.candidate_configuration_fingerprint_sha256:
            raise ValueError("tuning application context candidate configuration does not match its fingerprint")
        _validate_names("state", self.state_names)
        _validate_names("control", self.control_names)
        _validate_names("integral output", self.integral_output_names, required=self.controller_method == "lqi")
        if len(self.state_scales) != len(self.state_names) or len(self.control_scales) != len(self.control_names):
            raise ValueError("tuning application context scale counts must match its named coordinates")
        if any(not math.isfinite(value) or value <= 0.0 for value in (*self.state_scales, *self.control_scales)):
            raise ValueError("tuning application context scales must be finite and positive")
        if self.controller_method not in {"lqr", "lqi"}:
            raise ValueError("tuning application context controller method must be 'lqr' or 'lqi'")
        if self.controller_method == "lqr" and self.integral_output_names:
            raise ValueError("an LQR application context cannot declare integral outputs")
        if self.cache_hit and not self.cache_persisted:
            raise ValueError("a tuning application context cannot report a cache hit without a persistent cache")
        expected_gain_keys = (
            {"state_gain"}
            if self.controller_method == "lqr"
            else {"output_matrix", "state_gain", "integral_gain"}
        )
        if set(self.resolved_gains) != expected_gain_keys:
            raise ValueError(
                "tuning application context resolved gains must contain exactly "
                f"{sorted(expected_gain_keys)!r} for {self.controller_method!r}"
            )
        _validate_matrix(
            self.resolved_gains["state_gain"],
            rows=len(self.control_names),
            columns=len(self.state_names),
            label="state gain",
        )
        if self.controller_method == "lqi":
            _validate_matrix(
                self.resolved_gains["output_matrix"],
                rows=len(self.integral_output_names),
                columns=len(self.state_names),
                label="LQI output matrix",
            )
            _validate_matrix(
                self.resolved_gains["integral_gain"],
                rows=len(self.control_names),
                columns=len(self.integral_output_names),
                label="integral gain",
            )
        object.__setattr__(self, "weights", {key: tuple(values) for key, values in self.weights.items()})
        object.__setattr__(
            self,
            "resolved_gains",
            {key: tuple(tuple(value for value in row) for row in matrix) for key, matrix in self.resolved_gains.items()},
        )
        object.__setattr__(self, "candidate_configuration", _json_copy(self.candidate_configuration))
        ####

    @property
    def cache_disposition(self) -> str:
        """Return whether the candidate came from the persistent tuning cache."""

        return "hit" if self.cache_hit else "miss" if self.cache_persisted else "not_persisted"
        ####

    @property
    def resolved_gain_fingerprint_sha256(self) -> str:
        """Fingerprint the exact ordered gain matrices a runtime must apply."""

        return canonical_json_fingerprint(
            {
                "controller_method": self.controller_method,
                "state_names": list(self.state_names),
                "control_names": list(self.control_names),
                "integral_output_names": list(self.integral_output_names),
                "resolved_gains": {key: [list(row) for row in matrix] for key, matrix in self.resolved_gains.items()},
            }
        )
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the complete portable controller-application contract."""

        return {
            "schema": "taoryx.tuning-application-context/v1alpha1",
            "campaign_id": self.campaign_id,
            "node_id": self.node_id,
            "candidate_profile_id": self.candidate_profile_id,
            "candidate_configuration_fingerprint_sha256": self.candidate_configuration_fingerprint_sha256,
            "candidate_payload_fingerprint_sha256": self.candidate_payload_fingerprint_sha256,
            "resolved_gain_fingerprint_sha256": self.resolved_gain_fingerprint_sha256,
            "cache": {
                "key": self.cache_key,
                "disposition": self.cache_disposition,
            },
            "controller": {
                "method": self.controller_method,
                "state_names": list(self.state_names),
                "control_names": list(self.control_names),
                "integral_output_names": list(self.integral_output_names),
                "state_scales": list(self.state_scales),
                "control_scales": list(self.control_scales),
                "weights": {key: list(values) for key, values in self.weights.items()},
                "resolved_gains": {key: [list(row) for row in matrix] for key, matrix in self.resolved_gains.items()},
            },
            "candidate_configuration": _json_copy(self.candidate_configuration),
            "claim_boundary": (
                "This context carries one selected tuning-campaign candidate for a coordinate-compatible runtime. "
                "It is not proof that a vehicle batch instantiated or applied the gains; only a runtime binding "
                "emitted after application can make that claim."
            ),
        }
        ####

    def require_runtime_compatibility(
        self,
        *,
        controller_method: str,
        state_names: Sequence[str],
        control_names: Sequence[str],
        integral_output_names: Sequence[str] = (),
    ) -> None:
        """Fail closed unless a proposed runtime has the exact candidate coordinates.

        The comparison is order-sensitive because each gain matrix is ordered
        by these names.  A runtime with a projection, a different allocator,
        or a hand-selected controller must not claim this context was applied.
        """

        if controller_method != self.controller_method:
            raise ValueError(
                f"tuned {self.controller_method!r} context cannot bind a {controller_method!r} runtime"
            )
        expected = {
            "state names": self.state_names,
            "control names": self.control_names,
            "integral output names": self.integral_output_names,
        }
        observed = {
            "state names": tuple(state_names),
            "control names": tuple(control_names),
            "integral output names": tuple(integral_output_names),
        }
        for label, values in expected.items():
            if observed[label] != values:
                raise ValueError(
                    f"tuning application context {label} do not match the runtime: "
                    f"expected {values!r}, got {observed[label]!r}"
                )
        ####

    def runtime_binding_after_application(
        self,
        *,
        controller_method: str,
        state_names: Sequence[str],
        control_names: Sequence[str],
        integral_output_names: Sequence[str] = (),
    ) -> RuntimeTuningBindingReceipt:
        """Return a runtime receipt after the caller verifies actual application.

        This does not build or mutate a controller.  Its compatibility check
        makes it impossible to use the receipt for a differently ordered or
        projected controller by accident; the caller remains responsible for
        emitting it only after using ``resolved_gains`` to instantiate the
        compatible runtime controller.
        """

        self.require_runtime_compatibility(
            controller_method=controller_method,
            state_names=state_names,
            control_names=control_names,
            integral_output_names=integral_output_names,
        )
        return RuntimeTuningBindingReceipt(
            campaign_id=self.campaign_id,
            node_id=self.node_id,
            candidate_profile_id=self.candidate_profile_id,
            candidate_configuration_fingerprint_sha256=self.candidate_configuration_fingerprint_sha256,
            applied_gain_fingerprint_sha256=self.resolved_gain_fingerprint_sha256,
            controller_method=self.controller_method,
            cache_key=self.cache_key,
        )
        ####

    ####


@dataclass(frozen=True, slots=True)
class TuningApplicationContextSet:
    """One complete, non-ambiguous selection from a multi-node campaign.

    A single context is sufficient for a local controller screen.  A discrete
    schedule, however, must carry the selected candidate for *every* node it
    executes.  This contract prevents an executor from receiving a loose list
    and silently choosing one node, while preserving the existing singular
    context path for local screens.
    """

    contexts: tuple[TuningApplicationContext, ...]

    def __post_init__(self) -> None:
        if not self.contexts:
            raise ValueError("tuning application context sets require at least one context")
        campaign_ids = {context.campaign_id for context in self.contexts}
        if len(campaign_ids) != 1:
            raise ValueError("tuning application context sets must select one campaign")
        node_ids = tuple(context.node_id for context in self.contexts)
        if len(node_ids) != len(set(node_ids)):
            raise ValueError("tuning application context sets must contain one context per node")
        methods = {context.controller_method for context in self.contexts}
        if len(methods) != 1:
            raise ValueError("tuning application context sets must contain one controller method")
        ####

    @property
    def campaign_id(self) -> str:
        """Return the one campaign represented by every selected context."""

        return self.contexts[0].campaign_id
        ####

    @property
    def controller_method(self) -> TuningControllerMethod:
        """Return the one controller method shared by the selected nodes."""

        return self.contexts[0].controller_method
        ####

    @property
    def node_ids(self) -> tuple[str, ...]:
        """Return selected node identities in execution order."""

        return tuple(context.node_id for context in self.contexts)
        ####

    @property
    def singular(self) -> TuningApplicationContext | None:
        """Return the sole context when this is a one-node selection."""

        return self.contexts[0] if len(self.contexts) == 1 else None
        ####

    def for_node(self, node_id: str) -> TuningApplicationContext:
        """Return exactly the candidate selected for one executed node."""

        for context in self.contexts:
            if context.node_id == node_id:
                return context
        raise KeyError(f"tuning application context set has no node {node_id!r}")
        ####

    def require_exact_nodes(self, node_ids: Sequence[str]) -> None:
        """Reject incomplete, extra, or differently ordered schedule selections."""

        expected = tuple(node_ids)
        if self.node_ids != expected:
            raise ValueError(
                "tuning application context set nodes do not match the runtime schedule: "
                f"expected {expected!r}, got {self.node_ids!r}"
            )
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the complete portable selection and each exact candidate."""

        return {
            "schema": "taoryx.tuning-application-context-set/v1alpha1",
            "campaign_id": self.campaign_id,
            "controller_method": self.controller_method,
            "node_ids": list(self.node_ids),
            "contexts": [context.as_dict() for context in self.contexts],
            "claim_boundary": (
                "This set selects one exact candidate for each declared node. It is not proof that a runtime applied "
                "every candidate; the runtime must emit one binding receipt per executed node."
            ),
        }
        ####

    ####


def tuning_application_contexts_from_campaign_payload(
    payload: Mapping[str, object],
    *,
    campaign_id: str,
    cache_key: str,
    cache_hit: bool,
    cache_persisted: bool,
) -> tuple[TuningApplicationContext, ...]:
    """Extract every selected, gain-resolved candidate from a cached campaign.

    The campaign cache stays an audit artifact.  This function projects its
    selected candidates into a small runtime-facing contract without
    re-solving the plant or silently choosing a non-best profile.
    """

    if payload.get("schema") != "taoryx.tuning-campaign/v1alpha1":
        raise ValueError("tuning application context requires a tuning-campaign/v1alpha1 payload")
    if payload.get("status") != "candidate_ready":
        raise ValueError("tuning application context requires a candidate_ready campaign payload")
    campaign = _mapping(payload.get("campaign"), "campaign")
    payload_campaign_id = _string(campaign.get("campaign_id"), "campaign campaign_id")
    if payload_campaign_id != campaign_id:
        raise ValueError(
            f"tuning application context campaign {payload_campaign_id!r} does not match registration {campaign_id!r}"
        )
    nodes = payload.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        raise ValueError("candidate-ready tuning campaign has no node results")
    payload_fingerprint = canonical_json_fingerprint(payload)
    contexts: list[TuningApplicationContext] = []
    for raw_node in nodes:
        node = _mapping(raw_node, "campaign node")
        node_id = _string(node.get("node_id"), "campaign node_id")
        if node.get("status") != "candidate_ready":
            raise ValueError(f"candidate-ready campaign node {node_id!r} is not candidate_ready")
        report = _mapping(node.get("lqr"), f"campaign node {node_id!r} LQR report")
        profile_id = _string(report.get("best_profile_id"), f"campaign node {node_id!r} best_profile_id")
        candidates = report.get("candidates")
        if not isinstance(candidates, list):
            raise ValueError(f"campaign node {node_id!r} has no candidate list")
        candidate = next(
            (
                item
                for item in candidates
                if isinstance(item, Mapping) and item.get("profile_id") == profile_id
            ),
            None,
        )
        if candidate is None:
            raise ValueError(f"campaign node {node_id!r} best profile {profile_id!r} is not present")
        contexts.append(
            _context_from_candidate(
                candidate,
                campaign_id=campaign_id,
                node_id=node_id,
                cache_key=cache_key,
                cache_hit=cache_hit,
                cache_persisted=cache_persisted,
                payload_fingerprint=payload_fingerprint,
            )
        )
    return tuple(contexts)
    ####


def canonical_json_fingerprint(payload: object) -> str:
    """Return a stable SHA-256 fingerprint for one JSON-compatible payload."""

    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _context_from_candidate(
    candidate: Mapping[str, object],
    *,
    campaign_id: str,
    node_id: str,
    cache_key: str,
    cache_hit: bool,
    cache_persisted: bool,
    payload_fingerprint: str,
) -> TuningApplicationContext:
    """Validate one selected generic candidate and expose its resolved gains."""

    profile_id = _string(candidate.get("profile_id"), "candidate profile_id")
    raw_method = candidate.get("method")
    if raw_method == "lqr":
        method: TuningControllerMethod = "lqr"
    elif raw_method == "lqi":
        method = "lqi"
    else:
        raise ValueError(f"candidate {profile_id!r} has unsupported controller method {raw_method!r}")
    states = _string_tuple(candidate.get("state_names"), "candidate state_names")
    controls = _string_tuple(candidate.get("control_names"), "candidate control_names")
    integral_outputs = _string_tuple(candidate.get("integral_output_names"), "candidate integral_output_names", required=method == "lqi")
    if method == "lqr" and integral_outputs:
        raise ValueError(f"LQR candidate {profile_id!r} unexpectedly declares integral outputs")
    state_scales = _float_tuple(candidate.get("state_scales"), "candidate state_scales")
    control_scales = _float_tuple(candidate.get("control_scales"), "candidate control_scales")
    weights_payload = _mapping(candidate.get("weights"), "candidate weights")
    weights = {key: _float_tuple(value, f"candidate weights.{key}", required=False) for key, value in weights_payload.items()}
    gain_key = "lqr_controller" if method == "lqr" else "lqi_controller"
    raw_gains = candidate.get(gain_key)
    if not isinstance(raw_gains, Mapping):
        raise ValueError(f"candidate {profile_id!r} does not expose resolved gains in {gain_key!r}")
    gains_payload = raw_gains
    required_gain_keys = ("state_gain",) if method == "lqr" else ("output_matrix", "state_gain", "integral_gain")
    missing = [key for key in required_gain_keys if key not in gains_payload]
    if missing:
        raise ValueError(f"candidate {profile_id!r} does not expose resolved gains: {', '.join(missing)}")
    resolved_gains = {
        key: _matrix(gains_payload[key], f"candidate {gain_key}.{key}")
        for key in required_gain_keys
    }
    return TuningApplicationContext(
        campaign_id=campaign_id,
        node_id=node_id,
        candidate_profile_id=profile_id,
        candidate_configuration_fingerprint_sha256=canonical_json_fingerprint(candidate),
        candidate_payload_fingerprint_sha256=payload_fingerprint,
        controller_method=method,
        state_names=states,
        control_names=controls,
        integral_output_names=integral_outputs,
        state_scales=state_scales,
        control_scales=control_scales,
        weights=weights,
        resolved_gains=resolved_gains,
        candidate_configuration=dict(candidate),
        cache_key=cache_key,
        cache_hit=cache_hit,
        cache_persisted=cache_persisted,
    )
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    """Return a JSON object or explain the invalid campaign field."""

    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value
    ####


def _string(value: object, label: str) -> str:
    """Return one required non-empty string."""

    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a non-empty string")
    return value
    ####


def _string_tuple(value: object, label: str, *, required: bool = True) -> tuple[str, ...]:
    """Return a sequence of unique non-empty coordinate names."""

    if not isinstance(value, Sequence) or isinstance(value, str):
        raise ValueError(f"{label} must be a sequence of strings")
    values = tuple(_string(item, label) for item in value)
    _validate_names(label, values, required=required)
    return values
    ####


def _float_tuple(value: object, label: str, *, required: bool = True) -> tuple[float, ...]:
    """Return finite numeric campaign vectors without accepting booleans."""

    if not isinstance(value, Sequence) or isinstance(value, str):
        raise ValueError(f"{label} must be a numeric sequence")
    values: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)):
            raise ValueError(f"{label} must contain finite numbers")
        values.append(float(item))
    if required and not values:
        raise ValueError(f"{label} must not be empty")
    return tuple(values)
    ####


def _matrix(value: object, label: str) -> tuple[tuple[float, ...], ...]:
    """Return a finite rectangular numeric matrix from a campaign artifact."""

    if not isinstance(value, Sequence) or isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty numeric matrix")
    rows = tuple(_float_tuple(row, label) for row in value)
    columns = len(rows[0])
    if columns == 0 or any(len(row) != columns for row in rows):
        raise ValueError(f"{label} must be a rectangular non-empty matrix")
    return rows
    ####


def _validate_matrix(
    matrix: Sequence[Sequence[float]],
    *,
    rows: int,
    columns: int,
    label: str,
) -> None:
    """Validate one resolved gain matrix against its named dimensions."""

    if len(matrix) != rows or any(len(row) != columns for row in matrix):
        raise ValueError(f"tuning application context {label} has incompatible dimensions")
    if any(not math.isfinite(float(value)) for row in matrix for value in row):
        raise ValueError(f"tuning application context {label} must contain finite values")
    ####


def _validate_names(label: str, values: Sequence[str], *, required: bool = True) -> None:
    """Validate a named coordinate sequence."""

    if required and not values:
        raise ValueError(f"tuning application context requires at least one {label} name")
    if any(not isinstance(value, str) or not value.strip() for value in values) or len(set(values)) != len(values):
        raise ValueError(f"tuning application context {label} names must be unique non-empty strings")
    ####


def _validate_sha256(value: str, label: str) -> None:
    """Validate one lowercase SHA-256 fingerprint."""

    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ValueError(f"tuning application context {label} fingerprint must be a lowercase SHA-256 digest")
    ####


def _json_copy(value: Mapping[str, object]) -> dict[str, object]:
    """Copy a JSON-compatible mapping while rejecting NaN and non-JSON values."""

    encoded = json.dumps(value, sort_keys=True, allow_nan=False)
    decoded = json.loads(encoded)
    if not isinstance(decoded, dict):
        raise ValueError("tuning application context candidate configuration is not a JSON object")
    return deepcopy(decoded)
    ####


__all__ = [
    "RuntimeTuningBindingReceipt",
    "TuningApplicationContext",
    "TuningApplicationContextSet",
    "TuningControllerMethod",
    "canonical_json_fingerprint",
    "tuning_application_contexts_from_campaign_payload",
]
