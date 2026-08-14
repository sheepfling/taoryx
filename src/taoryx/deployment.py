"""Typed composition bindings for independently propagated child objects.

Parent vehicle plug-ins advertise that a deployment can occur.  A composition
may then select an independently installable child runtime for that deployment.
This module owns the narrow host contract between those two choices: it does
not infer a release impulse, coordinate transform, child model, or parent
qualification claim.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, Protocol, cast

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .fidelity_contracts import FidelityTier

if TYPE_CHECKING:
    from .plugins import PluginCatalog

DeploymentFrame = Literal["local_tangent", "eci"]
DeploymentStateTransfer = Literal[
    "inherited_accepted_state",
    "source_replay_kinematic_projection",
    "plugin_defined",
]


class DeploymentResolutionError(ValueError):
    """Raised when a selected child binding cannot be executed safely."""


class DeploymentChildReference(BaseModel):
    """Explicit plug-in-owned child runtime selected by a composition."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    plugin_id: str = Field(min_length=1)
    runtime_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    family_id: str = Field(min_length=1)
    fidelity: FidelityTier
    ####


class DeploymentBinding(BaseModel):
    """One composition-time connection from a parent deployment to a child runtime.

    A binding is intentionally optional at the parent-model level.  Its
    presence means the caller chose a child plug-in, model, fidelity, and
    transfer policy; its absence leaves a declared parent deployment as a
    lineage-only event rather than silently creating a child trajectory.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    deployment_id: str = Field(min_length=1)
    release_event_id: str = Field(min_length=1)
    child_object_id: str = Field(min_length=1)
    child: DeploymentChildReference
    state_transfer: DeploymentStateTransfer
    claim_boundary: str = Field(min_length=1)
    ####


class DeploymentReleaseState(BaseModel):
    """Accepted child initial state in the frame understood by its runtime."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    frame: DeploymentFrame
    time_s: float
    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    attitude_quaternion_wxyz: tuple[float, float, float, float] | None = None
    angular_rate_body_rad_s: tuple[float, float, float] | None = None
    provenance: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_finite_state(self) -> DeploymentReleaseState:
        values: tuple[float, ...] = (*self.position_m, *self.velocity_m_s, self.time_s)
        if self.attitude_quaternion_wxyz is not None:
            values = (*values, *self.attitude_quaternion_wxyz)
        if self.angular_rate_body_rad_s is not None:
            values = (*values, *self.angular_rate_body_rad_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("deployment release state values must be finite")
        if self.attitude_quaternion_wxyz is not None and math.isclose(
            sum(value * value for value in self.attitude_quaternion_wxyz),
            0.0,
            abs_tol=1.0e-15,
        ):
            raise ValueError("deployment release attitude quaternion must be nonzero")
        return self
        ####

    ####


class DeploymentReleaseRequest(BaseModel):
    """One parent-owned event handed to the selected child runtime."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    binding: DeploymentBinding
    parent_plugin_id: str = Field(min_length=1)
    parent_model_id: str = Field(min_length=1)
    parent_family_id: str = Field(min_length=1)
    parent_composition_id: str = Field(min_length=1)
    parent_object_id: str = Field(min_length=1)
    release_event_id: str = Field(min_length=1)
    release_state: DeploymentReleaseState
    parent_claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_event_identity(self) -> DeploymentReleaseRequest:
        if self.binding.release_event_id != self.release_event_id:
            raise ValueError(
                f"deployment release request event does not match the composition binding ({self.release_event_id!r} != {self.binding.release_event_id!r})"
            )
        return self
        ####

    ####


class DeploymentChildExecution(BaseModel):
    """Portable child-execution receipt returned to the parent runtime."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(default="taoryx.deployment-child-execution/v1alpha1", alias="schema", serialization_alias="schema")
    status: Literal["completed", "terminated", "invalid"]
    binding_id: str = Field(min_length=1)
    deployment_id: str = Field(min_length=1)
    relationship_kind: Literal["release", "separation", "deployment"]
    state_transfer: DeploymentStateTransfer
    parent_plugin_id: str = Field(min_length=1)
    parent_model_id: str = Field(min_length=1)
    parent_family_id: str = Field(min_length=1)
    parent_composition_id: str = Field(min_length=1)
    parent_object_id: str = Field(min_length=1)
    child_object_id: str = Field(min_length=1)
    parent_event_id: str = Field(min_length=1)
    runtime_plugin_id: str = Field(min_length=1)
    runtime_id: str = Field(min_length=1)
    child_model_id: str = Field(min_length=1)
    child_family_id: str = Field(min_length=1)
    requested_fidelity: FidelityTier
    realized_fidelity: str = Field(min_length=1)
    release_state: DeploymentReleaseState
    termination: str = Field(min_length=1)
    telemetry: tuple[dict[str, Any], ...] = ()
    provenance: dict[str, Any] = Field(default_factory=dict)
    claim_boundary: str = Field(min_length=1)
    ####


class DeploymentChildRuntime(Protocol):
    """A plug-in-owned child propagator callable through the common host."""

    id: str

    def supports(self, binding: DeploymentBinding) -> bool:
        """Return whether this runtime implements exactly ``binding``."""

        ...

    def execute(self, request: DeploymentReleaseRequest) -> DeploymentChildExecution:
        """Propagate one accepted child release and return its receipt."""

        ...


@dataclass(frozen=True, slots=True)
class DeploymentChildRuntimeRegistry:
    """Exact registry of independently installable child runtimes."""

    runtimes: tuple[DeploymentChildRuntime, ...]

    def __post_init__(self) -> None:
        identifiers = tuple(_runtime_identifier(item) for item in self.runtimes)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("deployment child runtime registry contains duplicate runtime IDs")
        ####

    def resolve(self, runtime_id: str) -> DeploymentChildRuntime:
        """Return one runtime without family or model fallback."""

        for runtime in self.runtimes:
            if _runtime_identifier(runtime) == runtime_id:
                return runtime
        raise KeyError(f"unknown deployment child runtime {runtime_id!r}")
        ####

    @property
    def ids(self) -> tuple[str, ...]:
        """Return registered runtime identities in plug-in declaration order."""

        return tuple(_runtime_identifier(item) for item in self.runtimes)
        ####

    ####


def deployment_binding_for(
    bindings: tuple[DeploymentBinding, ...],
    deployment_id: str,
) -> DeploymentBinding | None:
    """Return the selected binding for one parent deployment, if any."""

    matches = tuple(item for item in bindings if item.deployment_id == deployment_id)
    if len(matches) > 1:
        raise DeploymentResolutionError(f"deployment {deployment_id!r} has multiple selected child bindings")
    return matches[0] if matches else None
    ####


def validate_deployment_bindings(bindings: tuple[DeploymentBinding, ...]) -> tuple[DeploymentBinding, ...]:
    """Reject ambiguous binding identities before a native runtime is selected."""

    ids = tuple(item.id for item in bindings)
    deployments = tuple(item.deployment_id for item in bindings)
    child_objects = tuple(item.child_object_id for item in bindings)
    for label, values in (("binding IDs", ids), ("deployment IDs", deployments), ("child object IDs", child_objects)):
        if len(values) != len(set(values)):
            raise ValueError(f"deployment bindings contain duplicate {label}")
    return bindings
    ####


def execute_deployment_child(
    request: DeploymentReleaseRequest,
    *,
    plugins: PluginCatalog | None = None,
) -> DeploymentChildExecution:
    """Resolve and execute one selected child runtime through one focused catalog.

    ``plugins`` remains a type-only dependency so this core contract does not
    import plug-in discovery until execution. A parent batch runtime normally
    inherits the catalog scope selected by the common batch host.
    """

    from .plugins import current_plugin_catalog, discover_plugins

    catalog = plugins if plugins is not None else current_plugin_catalog()
    if catalog is None:
        catalog = discover_plugins()
    contribution = None
    try:
        contribution = catalog.contribution("deployment_child_runtime", request.binding.child.runtime_id)
    except KeyError as error:
        raise DeploymentResolutionError(
            f"deployment binding {request.binding.id!r} requires child runtime "
            f"{request.binding.child.runtime_id!r} from plug-in {request.binding.child.plugin_id!r}, "
            "but it is not present in the selected plug-in catalog"
        ) from error
    if contribution.plugin.id != request.binding.child.plugin_id:
        raise DeploymentResolutionError(
            f"deployment binding {request.binding.id!r} requires plug-in {request.binding.child.plugin_id!r}, "
            f"but runtime {request.binding.child.runtime_id!r} is owned by {contribution.plugin.id!r}"
        )
    runtime = cast(DeploymentChildRuntime, contribution.value)
    if not callable(getattr(runtime, "supports", None)) or not callable(getattr(runtime, "execute", None)):
        raise DeploymentResolutionError(f"deployment child runtime {request.binding.child.runtime_id!r} does not implement the required runtime contract")
    if not runtime.supports(request.binding):
        raise DeploymentResolutionError(f"deployment child runtime {request.binding.child.runtime_id!r} does not support binding {request.binding.id!r}")
    execution = runtime.execute(request)
    _validate_execution_receipt(execution, request, contribution.plugin.id)
    return execution
    ####


def _runtime_identifier(runtime: object) -> str:
    identifier = getattr(runtime, "id", None)
    if not isinstance(identifier, str) or not identifier.strip():
        raise TypeError("deployment child runtimes require a non-empty string id")
    return identifier
    ####


def _validate_execution_receipt(
    execution: DeploymentChildExecution,
    request: DeploymentReleaseRequest,
    plugin_id: str,
) -> None:
    """Ensure a child runtime cannot relabel its selected parent contract."""

    expected = request.binding
    observed = (
        execution.binding_id,
        execution.deployment_id,
        execution.state_transfer,
        execution.parent_plugin_id,
        execution.parent_model_id,
        execution.parent_family_id,
        execution.parent_composition_id,
        execution.parent_object_id,
        execution.child_object_id,
        execution.parent_event_id,
        execution.runtime_plugin_id,
        execution.runtime_id,
        execution.child_model_id,
        execution.child_family_id,
        execution.requested_fidelity,
    )
    required = (
        expected.id,
        expected.deployment_id,
        expected.state_transfer,
        request.parent_plugin_id,
        request.parent_model_id,
        request.parent_family_id,
        request.parent_composition_id,
        request.parent_object_id,
        expected.child_object_id,
        request.release_event_id,
        plugin_id,
        expected.child.runtime_id,
        expected.child.model_id,
        expected.child.family_id,
        expected.child.fidelity,
    )
    if observed != required:
        raise DeploymentResolutionError(f"deployment child runtime {expected.child.runtime_id!r} returned a receipt that does not match its selected binding")
    ####


__all__ = [
    "DeploymentBinding",
    "DeploymentChildExecution",
    "DeploymentChildReference",
    "DeploymentChildRuntime",
    "DeploymentChildRuntimeRegistry",
    "DeploymentFrame",
    "DeploymentReleaseRequest",
    "DeploymentReleaseState",
    "DeploymentResolutionError",
    "DeploymentStateTransfer",
    "deployment_binding_for",
    "execute_deployment_child",
    "validate_deployment_bindings",
]
