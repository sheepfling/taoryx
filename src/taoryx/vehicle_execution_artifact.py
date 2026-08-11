"""Typed common envelope for provider-owned vehicle execution results.

Vehicle plug-ins are intentionally free to add family-specific telemetry and
diagnostics.  The common runtime must not, however, accept an arbitrary
dictionary and discover missing identity or disposition fields several stages
later.  This model owns the small cross-family envelope; provider payload
fields remain preserved as explicit Pydantic extras.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .vehicle_composition import CompiledVehicleComposition

VEHICLE_EXECUTION_PACKET_SCHEMA = "taoryx.vehicle-execution-packet/v1alpha1"
VEHICLE_EXECUTION_HOST_CONTEXT_SCHEMA = "taoryx.vehicle-execution-host-context/v1alpha1"
VEHICLE_EXECUTION_REQUEST_SCHEMA = "taoryx.vehicle-batch-factory-request/v1alpha1"

ExecutionOutcomeScope = Literal["mission", "local_screen"]
ExecutionDisposition = Literal["passed", "failed"]


class VehicleExecutionArtifact(BaseModel):
    """Validated common envelope returned by one native batch factory.

    ``extra='allow'`` is deliberate: plug-ins own their detailed runtime,
    envelope, traces, and evaluation fields.  The fields below are the
    non-negotiable handoff that every consumer needs before inspecting those
    extensions.
    """

    model_config = ConfigDict(
        frozen=True,
        extra="allow",
        populate_by_name=True,
    )

    schema_id: str = Field(alias="schema", min_length=1)
    status: str = Field(min_length=1)
    composition: Mapping[str, Any]
    claim_boundary: str = Field(min_length=1)
    output_dir: str | None = None
    mission_pass: bool | None = None
    screen_pass: bool | None = None

    @model_validator(mode="before")
    @classmethod
    def promote_legacy_disposition(cls, value: object) -> object:
        """Normalize established provider payloads at the one common seam.

        Older local screens keep their verdict inside ``control_screen`` or
        ``evaluation``.  This compatibility shim is intentionally centralized
        so downstream consumers no longer need to know those provider shapes.
        """

        if not isinstance(value, Mapping):
            return value
        payload = dict(value)
        if payload.get("mission_pass") is None and payload.get("screen_pass") is None:
            for field in ("control_screen", "evaluation", "truth_evaluation"):
                nested = payload.get(field)
                if isinstance(nested, Mapping):
                    if isinstance(nested.get("mission_pass"), bool):
                        payload["mission_pass"] = nested["mission_pass"]
                        break
                    if isinstance(nested.get("screen_pass"), bool):
                        payload["screen_pass"] = nested["screen_pass"]
                        break
        return payload
        ####

    @model_validator(mode="after")
    def validate_disposition(self) -> VehicleExecutionArtifact:
        """Require one unambiguous pass/fail disposition from every factory."""

        if self.mission_pass is None and self.screen_pass is None:
            raise ValueError("execution artifact requires mission_pass or screen_pass")
        if self.mission_pass is not None and self.screen_pass is not None and self.mission_pass != self.screen_pass:
            raise ValueError("execution artifact mission_pass and screen_pass disagree")
        return self
        ####

    @property
    def passed(self) -> bool:
        """Return the one common pass disposition without key probing."""

        if self.mission_pass is not None:
            return self.mission_pass
        assert self.screen_pass is not None
        return self.screen_pass
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the normalized common envelope with provider extras intact."""

        return self.model_dump(mode="json", by_alias=True)
        ####

    @classmethod
    def from_payload(
        cls,
        payload: Mapping[str, object],
        *,
        composition: CompiledVehicleComposition,
    ) -> VehicleExecutionArtifact:
        """Validate a plug-in payload and bind it to the requested composition."""

        artifact = cls.model_validate(payload)
        compiled = CompiledVehicleComposition.model_validate(artifact.composition)
        if compiled.id != composition.id:
            raise ValueError(
                "execution artifact composition ID disagrees with the requested composition: "
                f"{compiled.id!r} != {composition.id!r}"
            )
        if compiled.identity_sha256 != composition.identity_sha256:
            raise ValueError("execution artifact composition fingerprint disagrees with the requested composition")
        return artifact
        ####

    ####


class VehicleExecutionTuningApplicationRecord(BaseModel):
    """Portable identity for one exact controller candidate application.

    The full gain remains owned by the in-process tuning context and is never
    serialized into a generic execution request.  This compact record gives
    persisted execution evidence enough information to prove which cached,
    compatibility-checked candidate the plug-in consumed.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    campaign_id: str = Field(min_length=1)
    node_id: str = Field(min_length=1)
    candidate_profile_id: str = Field(min_length=1)
    candidate_configuration_fingerprint_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    resolved_gain_fingerprint_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    ####


class VehicleExecutionTuningApplicationSetRecord(BaseModel):
    """Portable request identity for every candidate applied by a discrete schedule."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    campaign_id: str = Field(min_length=1)
    controller_method: Literal["lqr", "lqi"]
    node_ids: tuple[str, ...]
    contexts: tuple[VehicleExecutionTuningApplicationRecord, ...]

    @model_validator(mode="after")
    def coherent_complete_node_selection(self) -> VehicleExecutionTuningApplicationSetRecord:
        """Require a complete ordered node-to-candidate selection."""

        if len(self.contexts) < 2:
            raise ValueError("execution tuning application context sets require at least two contexts")
        if any(not node_id.strip() for node_id in self.node_ids) or len(self.node_ids) != len(set(self.node_ids)):
            raise ValueError("execution tuning application context set node IDs must be unique non-empty strings")
        if tuple(context.node_id for context in self.contexts) != self.node_ids:
            raise ValueError("execution tuning application context set nodes must match its ordered contexts")
        if any(context.campaign_id != self.campaign_id for context in self.contexts):
            raise ValueError("execution tuning application context set contexts must identify its campaign")
        return self
        ####

    ####


class VehicleExecutionRequestRecord(BaseModel):
    """Portable identity for a host-to-plug-in batch invocation.

    The request is deliberately a narrow projection rather than another copy
    of the compiled composition.  The packet stores the full immutable
    composition once and this record binds it to the exact registered factory.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.vehicle-batch-factory-request/v1alpha1"] = Field(alias="schema")
    composition_id: str = Field(min_length=1)
    composition_identity_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    family_id: str = Field(min_length=1)
    vehicle_id: str = Field(min_length=1)
    mission_id: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    factory_id: str = Field(min_length=1)
    execution_mode: str = Field(min_length=1)
    output_dir: str = Field(min_length=1)
    max_steps: int | None = Field(default=None, gt=0)
    tuning_application_context: VehicleExecutionTuningApplicationRecord | None = None
    tuning_application_context_set: VehicleExecutionTuningApplicationSetRecord | None = None

    @model_validator(mode="after")
    def unambiguous_tuning_application(self) -> VehicleExecutionRequestRecord:
        """Reject host packets that blur a local candidate and a schedule selection."""

        if self.tuning_application_context is not None and self.tuning_application_context_set is not None:
            raise ValueError("execution request accepts either one tuning application context or one context set, not both")
        return self
        ####

    ####


class VehicleExecutionOutcome(BaseModel):
    """One unambiguous common result disposition.

    ``provider_status`` remains intentionally descriptive: vehicle families
    can retain useful distinctions such as source-replay or local-screen
    outcomes without forcing every plug-in into one physical qualification
    taxonomy.  Common consumers should use ``scope`` and ``disposition``.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    scope: ExecutionOutcomeScope
    disposition: ExecutionDisposition
    provider_status: str = Field(min_length=1)
    ####

    @property
    def passed(self) -> bool:
        """Return the Boolean projection needed by legacy command surfaces."""

        return self.disposition == "passed"
        ####

    @classmethod
    def from_artifact(cls, artifact: VehicleExecutionArtifact) -> VehicleExecutionOutcome:
        """Project the established result fields into one typed disposition."""

        if artifact.mission_pass is not None:
            return cls(
                scope="mission",
                disposition="passed" if artifact.mission_pass else "failed",
                provider_status=artifact.status,
            )
        assert artifact.screen_pass is not None
        return cls(
            scope="local_screen",
            disposition="passed" if artifact.screen_pass else "failed",
            provider_status=artifact.status,
        )
        ####

    ####


class VehicleExecutionHostContext(BaseModel):
    """Host-owned identity attached after a provider returns its artifact."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.vehicle-execution-host-context/v1alpha1"] = Field(alias="schema")
    request: VehicleExecutionRequestRecord
    interface_id: str = Field(min_length=1)
    interface_fingerprint_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    provider_execution_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    preflight_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    ####


class VehicleExecutionPacket(BaseModel):
    """Canonical persisted result packet for one composed batch execution.

    The packet intentionally remains flat for established catalog consumers:
    provider-owned ``runtime``, ``control_screen``, and telemetry summaries
    remain available at the top level as additive Pydantic extras.  Their
    unmodified provider payload is also preserved under ``provider_execution``
    so agents can diagnose the exact plug-in handoff without depending on a
    second raw sidecar.
    """

    model_config = ConfigDict(frozen=True, extra="allow", populate_by_name=True)

    schema_id: Literal["taoryx.vehicle-execution-packet/v1alpha1"] = Field(alias="schema")
    status: str = Field(min_length=1)
    composition: Mapping[str, Any]
    claim_boundary: str = Field(min_length=1)
    provider_execution_schema: str = Field(min_length=1)
    provider_claim_boundary: str = Field(min_length=1)
    provider_execution: VehicleExecutionArtifact
    host_execution: VehicleExecutionHostContext
    outcome: VehicleExecutionOutcome
    packet_identity_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    output_dir: str | None = None
    mission_pass: bool | None = None
    screen_pass: bool | None = None

    @model_validator(mode="after")
    def validate_packet_identity(self) -> VehicleExecutionPacket:
        """Reject a packet whose flattened or nested facts disagree."""

        compiled = CompiledVehicleComposition.model_validate(self.composition)
        request = self.host_execution.request
        if compiled.id != request.composition_id:
            raise ValueError("execution packet request composition ID disagrees with packet composition")
        if compiled.identity_sha256 != request.composition_identity_sha256:
            raise ValueError("execution packet request composition fingerprint disagrees with packet composition")
        if compiled.family_id != request.family_id or compiled.vehicle_id != request.vehicle_id:
            raise ValueError("execution packet request vehicle selection disagrees with packet composition")
        if compiled.mission != request.mission_id or compiled.fidelity != request.fidelity:
            raise ValueError("execution packet request mission selection disagrees with packet composition")
        expected_interface_id = f"{compiled.family_id}/{compiled.fidelity}"
        if self.host_execution.interface_id != expected_interface_id:
            raise ValueError("execution packet interface ID disagrees with packet composition")
        provider = VehicleExecutionArtifact.from_payload(
            self.provider_execution.as_dict(),
            composition=compiled,
        )
        if provider.schema_id != self.provider_execution_schema:
            raise ValueError("execution packet provider schema does not match the nested provider artifact")
        if provider.claim_boundary != self.provider_claim_boundary:
            raise ValueError("execution packet provider claim boundary does not match the nested provider artifact")
        if provider.status != self.status:
            raise ValueError("execution packet status does not match the nested provider artifact")
        if provider.mission_pass != self.mission_pass or provider.screen_pass != self.screen_pass:
            raise ValueError("execution packet legacy pass fields do not match the nested provider artifact")
        if VehicleExecutionOutcome.from_artifact(provider) != self.outcome:
            raise ValueError("execution packet outcome does not match the nested provider artifact")
        if _mapping_sha256(self.provider_execution.as_dict()) != self.host_execution.provider_execution_sha256:
            raise ValueError("execution packet provider artifact fingerprint is invalid")
        preflight = self.provider_execution.as_dict().get("preflight")
        expected_preflight_sha256 = _mapping_sha256(preflight) if isinstance(preflight, Mapping) else None
        if self.host_execution.preflight_sha256 != expected_preflight_sha256:
            raise ValueError("execution packet preflight fingerprint is invalid")
        if self.packet_identity_sha256 != _packet_identity_sha256(self._identity_payload()):
            raise ValueError("execution packet identity fingerprint is invalid")
        return self
        ####

    @classmethod
    def from_artifact(
        cls,
        artifact: VehicleExecutionArtifact,
        *,
        request: Mapping[str, object],
        interface_id: str,
        interface_fingerprint_sha256: str,
    ) -> VehicleExecutionPacket:
        """Attach host-owned identity while retaining all provider extensions."""

        request_record = VehicleExecutionRequestRecord.model_validate(request)
        provider_payload = artifact.as_dict()
        preflight = provider_payload.get("preflight")
        host_context = VehicleExecutionHostContext(
            schema="taoryx.vehicle-execution-host-context/v1alpha1",
            request=request_record,
            interface_id=interface_id,
            interface_fingerprint_sha256=interface_fingerprint_sha256,
            provider_execution_sha256=_mapping_sha256(provider_payload),
            preflight_sha256=_mapping_sha256(preflight) if isinstance(preflight, Mapping) else None,
        )
        packet_payload: dict[str, object] = {
            **provider_payload,
            "schema": VEHICLE_EXECUTION_PACKET_SCHEMA,
            "provider_execution_schema": artifact.schema_id,
            "provider_claim_boundary": artifact.claim_boundary,
            "provider_execution": provider_payload,
            "host_execution": host_context.model_dump(mode="json", by_alias=True),
            "outcome": VehicleExecutionOutcome.from_artifact(artifact).model_dump(mode="json"),
            "claim_boundary": (
                "This host-owned packet binds one provider execution result to the requested immutable composition, "
                "registered factory, and advertised interface. Provider-specific evidence remains preserved; this "
                "envelope does not establish numerical accuracy, physical-effector behavior, or qualification."
            ),
        }
        packet_payload["packet_identity_sha256"] = _packet_identity_sha256(packet_payload)
        return cls.model_validate(packet_payload)
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the portable canonical packet with provider extras intact."""

        return self.model_dump(mode="json", by_alias=True)
        ####

    def write_json(self, path: str | Path) -> Path:
        """Atomically replace the provider's provisional execution sidecar."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with NamedTemporaryFile("w", encoding="utf-8", dir=destination.parent, prefix=f".{destination.name}.", suffix=".tmp", delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(json.dumps(self.as_dict(), indent=2, sort_keys=True) + "\n")
        try:
            temporary.replace(destination)
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
        return destination
        ####

    def _identity_payload(self) -> dict[str, object]:
        payload = self.model_dump(mode="json", by_alias=True)
        payload.pop("packet_identity_sha256", None)
        return payload
        ####

    ####


def read_vehicle_execution_packet(path: str | Path) -> VehicleExecutionPacket:
    """Read and validate one host-owned canonical execution packet."""

    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"could not read execution packet {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise ValueError("execution packet must contain one JSON object")
    try:
        return VehicleExecutionPacket.model_validate(payload)
    except ValueError as error:
        raise ValueError(f"invalid execution packet {source}: {error}") from error
    ####


def _mapping_sha256(payload: Mapping[str, object]) -> str:
    """Return a canonical fingerprint for one JSON-safe evidence mapping."""

    try:
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise ValueError(f"execution evidence is not fingerprintable: {error}") from error
    return hashlib.sha256(encoded).hexdigest()
    ####


def _packet_identity_sha256(payload: Mapping[str, object]) -> str:
    """Fingerprint every flat and nested fact in a canonical packet."""

    return _mapping_sha256(payload)
    ####


__all__ = [
    "VEHICLE_EXECUTION_HOST_CONTEXT_SCHEMA",
    "VEHICLE_EXECUTION_PACKET_SCHEMA",
    "VEHICLE_EXECUTION_REQUEST_SCHEMA",
    "VehicleExecutionArtifact",
    "VehicleExecutionHostContext",
    "VehicleExecutionOutcome",
    "VehicleExecutionPacket",
    "VehicleExecutionTuningApplicationRecord",
    "VehicleExecutionTuningApplicationSetRecord",
    "VehicleExecutionRequestRecord",
    "read_vehicle_execution_packet",
]
