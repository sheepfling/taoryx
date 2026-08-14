"""Discoverable CADAC sensor ownership and migration boundaries."""

from __future__ import annotations

from typing import Literal

from pydantic import Field, model_validator

from .input_ast import CadacModel

CadacSensorExecution = Literal["native_projection_in_source_module", "source_only", "none"]
CadacSensorReadiness = Literal["available", "development", "blocked", "not_applicable"]


class CadacSensorIntegrationContract(CadacModel):
    """Sensor facts advertised beside one CADAC composition model.

    A native projection may run inside a source-ordered batch module. Only an
    advertised persistent session additionally owns the standard ``SensorBus``
    cadence, delivery, and checkpoint state.
    """

    schema_id: str = "taoryx.cadac-sensor-integration/v0alpha1"
    model_id: str = Field(min_length=1)
    status: CadacSensorReadiness
    native_sensor_provider: str | None = None
    execution: CadacSensorExecution
    sensor_bus_status: CadacSensorReadiness
    source_specialised_state: tuple[str, ...] = ()
    blockers: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_sensor_status(self) -> "CadacSensorIntegrationContract":
        if self.execution == "native_projection_in_source_module" and self.native_sensor_provider is None:
            raise ValueError("native CADAC sensor projection requires a declared Taoryx sensor provider")
        if self.status == "blocked" and not self.blockers:
            raise ValueError("blocked CADAC sensor integration requires explicit blockers")
        if self.execution == "none" and self.native_sensor_provider is not None:
            raise ValueError("models without a participating sensor cannot declare a native sensor provider")
        return self
        ####

    ####


_RELATIVE_STATE_MODELS: dict[str, tuple[str, ...]] = {
    "cadac.aim5.missile": (),
    "cadac.ads6.srbm": (),
    "cadac.sraam6.missile": ("source acquisition and seeker gimbal state", "source lock and filter state"),
    "cadac.agm6.missile": ("source acquisition and sensor gimbal state", "source lock, blind-range, and filter state"),
    "cadac.ads6.sam": ("source RF/IR gimbal state", "source acquisition, lock, and filtering state"),
    "cadac.ads6.engagement": ("source RADAR0 noise sequence", "source track-manager and launch scheduling state"),
    "cadac.ghame6.hypersonic_vehicle": (
        "source RADAR0 polar/noise measurement and update-cadence state",
        "source RADAR0 track-file behavior",
    ),
}

_PERSISTENT_SENSOR_BLOCKERS: dict[str, tuple[str, ...]] = {
    "cadac.ads6.sam": (
        "the standalone SAM surface owns only the physical plant; its source RF/IR controller requires live target truth and radar context from the ADS6 package scheduler",
        "a persistent native SensorBus must therefore be owned by cadac.ads6.engagement, not fabricated around a standalone direct-command plant",
    ),
}


def build_cadac_sensor_integration_contract(
    model_id: str,
    *,
    persistent_session: bool = False,
) -> CadacSensorIntegrationContract:
    """Return a strict sensor boundary for one published CADAC model ID."""

    specialised_state = _RELATIVE_STATE_MODELS.get(model_id)
    if specialised_state is not None:
        bus_status: CadacSensorReadiness = "available" if persistent_session else "blocked"
        blockers = (
            ()
            if persistent_session
            else _PERSISTENT_SENSOR_BLOCKERS.get(
                model_id,
                ("the CADAC batch runtime has no persistent session that can bind native sensor clocks, delivery, or SensorBus checkpoint state",),
            )
        )
        return CadacSensorIntegrationContract(
            model_id=model_id,
            status="available",
            native_sensor_provider="relative-state-track",
            execution="native_projection_in_source_module",
            sensor_bus_status=bus_status,
            source_specialised_state=specialised_state,
            blockers=blockers,
            claim_boundary=(
                "The source module is explicitly connected to a Taoryx typed raw relative-state observation at its committed boundary. "
                + (
                    "A persistent session also publishes the native SensorBus packet at committed source boundaries. "
                    if persistent_session
                    else "No persistent SensorBus delivery session is currently bound. "
                )
                + "CADAC-specific gimbal, acquisition, lock, filtering, and scheduling state remains explicit source-compatibility behavior."
            ),
        )
    return CadacSensorIntegrationContract(
        model_id=model_id,
        status="not_applicable",
        execution="none",
        sensor_bus_status="not_applicable",
        claim_boundary=(
            "This advertised CADAC model has no participating sensor path integrated with the current executable "
            "surface. Omitted or source-only modules are not advertised as native Taoryx sensors."
        ),
    )
    ####


__all__ = [
    "CadacSensorExecution",
    "CadacSensorIntegrationContract",
    "CadacSensorReadiness",
    "build_cadac_sensor_integration_contract",
]
####
