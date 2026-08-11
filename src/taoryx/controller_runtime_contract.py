"""Typed common declaration for a controller-owning vehicle runtime.

Vehicle-specific runtime records may carry any additional telemetry or
diagnostics.  Once a runtime claims a common LQR/LQI controller, however, the
method, realization, integral coordinates, and optional campaign receipt have
one cross-family meaning and must be validated at the execution boundary.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .tuning_application import RuntimeTuningBindingReceipt, TuningControllerMethod


class ControllerRuntimeDeclaration(BaseModel):
    """Portable controller metadata embedded in a provider runtime payload."""

    model_config = ConfigDict(frozen=True, extra="allow")

    controller_method: TuningControllerMethod
    control_realization: str = Field(min_length=1)
    integral_output_names: tuple[str, ...] = ()
    controller_id: str | None = Field(default=None, min_length=1)
    integrators_exercised: bool | None = None
    tuning_binding: RuntimeTuningBindingReceipt | None = None
    tuning_bindings: tuple[RuntimeTuningBindingReceipt, ...] = ()

    @model_validator(mode="after")
    def validate_controller_contract(self) -> ControllerRuntimeDeclaration:
        """Bind controller mode, integral coordinates, and tuning receipt."""

        if any(not name.strip() for name in self.integral_output_names):
            raise ValueError("execution integral output names must be nonempty strings")
        if len(self.integral_output_names) != len(set(self.integral_output_names)):
            raise ValueError("execution integral output names must be unique")
        if self.controller_method == "lqi" and not self.integral_output_names:
            raise ValueError("LQI execution must identify at least one integral output")
        if self.controller_method == "lqr" and self.integral_output_names:
            raise ValueError("LQR execution must not identify integral outputs")
        if self.tuning_binding is not None and self.tuning_bindings:
            raise ValueError("execution runtime accepts either one tuning binding or a binding set, not both")
        if self.tuning_bindings and len(self.tuning_bindings) < 2:
            raise ValueError("execution tuning binding sets require at least two node receipts")
        bindings = ((self.tuning_binding,) if self.tuning_binding is not None else self.tuning_bindings)
        node_ids = tuple(binding.node_id for binding in bindings)
        if len(node_ids) != len(set(node_ids)):
            raise ValueError("execution tuning binding sets must contain one receipt per node")
        campaign_ids = {binding.campaign_id for binding in bindings}
        if len(campaign_ids) > 1:
            raise ValueError("execution tuning binding sets must identify one campaign")
        for binding in bindings:
            binding.require_runtime_method(self.controller_method)
        return self
        ####

    @property
    def all_tuning_bindings(self) -> tuple[RuntimeTuningBindingReceipt, ...]:
        """Return every exact candidate receipt represented by this runtime."""

        return (self.tuning_binding,) if self.tuning_binding is not None else self.tuning_bindings
        ####

    ####


__all__ = ["ControllerRuntimeDeclaration"]
