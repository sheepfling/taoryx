"""Plug-in registry for executable controller-tuning campaigns.

The numerical campaign runner belongs to the Taoryx host.  A model plug-in
contributes only the family-specific adapter and declared campaign inputs
needed to run that shared sequence.  Keeping those two responsibilities
separate prevents a model package from growing its own incompatible tuning
loop while ensuring the host never invents trim points, state scales, or
control authority.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Callable, Mapping, Sequence
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .family_adapter import StandardFamilyAdapter
    from .trajectory.configuration_contract import ConfigurableTrajectoryProviderRegistry
    from .tuning_application import TuningApplicationContext
    from .tuning_campaign import TuningCampaign, TuningCampaignReport

ControllerTuningAdapterFactory = Callable[[], "StandardFamilyAdapter"]
ControllerTuningCampaignFactory = Callable[[], "TuningCampaign"]


@dataclass(frozen=True, slots=True)
class CachedTuningCampaignResult:
    """One tuning payload plus its content-addressed cache disposition."""

    payload: dict[str, Any]
    cache_key: str
    cache_hit: bool
    cache_path: Path | None

    ####


@dataclass(frozen=True, slots=True)
class ControllerTuningCampaignRegistration:
    """One model-owned campaign executable through the common host runner."""

    id: str
    provider_id: str
    model_id: str
    family_id: str
    fidelity: str
    realization_ids: tuple[str, ...]
    mission_template_ids: tuple[str, ...]
    description: str
    adapter_factory: ControllerTuningAdapterFactory
    campaign_factory: ControllerTuningCampaignFactory
    provider_aliases: tuple[str, ...] = ()
    local_controller_screens: tuple[Mapping[str, Any], ...] = ()
    required_extras: tuple[str, ...] = ("taoryx[scipy]",)
    claim_boundary: str = (
        "A registered campaign runs the common local controller-design screen. "
        "It does not by itself qualify nonlinear mission tracking, allocation, "
        "actuator dynamics, or an operating envelope."
    )

    def __post_init__(self) -> None:
        identities = (
            self.id,
            self.provider_id,
            self.model_id,
            self.family_id,
            self.fidelity,
            self.description,
            self.claim_boundary,
        )
        if not all(item.strip() for item in identities):
            raise ValueError("controller-tuning registrations require non-empty identity and descriptions")
        if not self.realization_ids:
            raise ValueError("controller-tuning registrations require at least one realization ID")
        if (
            len(self.provider_aliases) != len(set(self.provider_aliases))
            or any(not identifier.strip() for identifier in self.provider_aliases)
            or self.provider_id in self.provider_aliases
        ):
            raise ValueError("controller-tuning registration has invalid provider aliases")
        for label, values in (
            ("realization IDs", self.realization_ids),
            ("mission-template IDs", self.mission_template_ids),
            ("required extras", self.required_extras),
        ):
            if len(values) != len(set(values)) or any(not item.strip() for item in values):
                raise ValueError(f"controller-tuning registration has invalid {label}")
        screen_ids: set[str] = set()
        for screen in self.local_controller_screens:
            if not isinstance(screen, Mapping):
                raise TypeError("local controller screens must be mapping advertisements")
            required = {
                "schema",
                "id",
                "mission_template_id",
                "fidelity",
                "operations",
                "control_realization",
                "controller",
                "claim_boundary",
            }
            missing = sorted(required - set(screen))
            if missing:
                raise ValueError(f"local controller screen is missing: {', '.join(missing)}")
            identifier = screen["id"]
            mission_id = screen["mission_template_id"]
            if not isinstance(identifier, str) or not identifier.strip() or identifier in screen_ids:
                raise ValueError("local controller screen IDs must be nonempty and unique per campaign")
            if not isinstance(mission_id, str) or mission_id not in self.mission_template_ids:
                raise ValueError("local controller screen must select one registered campaign mission")
            if screen["fidelity"] != self.fidelity:
                raise ValueError("local controller screen fidelity must match its campaign")
            operations = screen["operations"]
            if not isinstance(operations, list) or not operations:
                raise ValueError("local controller screen must advertise one or more operations")
            if not all(isinstance(operation, str) and operation.strip() for operation in operations):
                raise ValueError("local controller screen operations must be nonempty strings")
            controller = screen["controller"]
            if not isinstance(controller, Mapping):
                raise ValueError("local controller screen controller declaration must be a mapping")
            method = controller.get("method")
            if method not in {"lqr", "lqi"}:
                raise ValueError("local controller screen controller method must be 'lqr' or 'lqi'")
            for timing_name in ("fixed_cadence_s", "screen_duration_s"):
                timing = controller.get(timing_name)
                if timing is not None and (
                    isinstance(timing, bool)
                    or not isinstance(timing, int | float)
                    or not math.isfinite(float(timing))
                    or float(timing) <= 0.0
                ):
                    raise ValueError(f"local controller screen {timing_name} must be a positive finite number")
            raw_integral_outputs = controller.get("integral_output_names", ())
            if not isinstance(raw_integral_outputs, Sequence) or isinstance(raw_integral_outputs, str):
                raise ValueError("local controller screen integral output names must be a sequence")
            integral_outputs = tuple(raw_integral_outputs)
            if (
                any(not isinstance(name, str) or not name.strip() for name in integral_outputs)
                or len(integral_outputs) != len(set(integral_outputs))
            ):
                raise ValueError("local controller screen integral output names must be unique nonempty strings")
            if method == "lqi":
                if controller.get("campaign_id") != self.id:
                    raise ValueError("local LQI controller screen must identify this campaign")
                if not integral_outputs:
                    raise ValueError("local LQI controller screen must advertise one or more integral outputs")
            elif integral_outputs:
                raise ValueError("local LQR controller screen cannot advertise integral outputs")
            screen_ids.add(identifier)
        ####

    def matches(
        self,
        *,
        provider_id: str,
        model_id: str,
        fidelity: str,
        realization_id: str | None = None,
        mission_template_id: str | None = None,
    ) -> bool:
        """Return whether this campaign applies to an advertised selection."""

        return (
            provider_id in (self.provider_id, *self.provider_aliases)
            and self.model_id == model_id
            and self.fidelity == fidelity
            and (realization_id is None or realization_id in self.realization_ids)
            and (mission_template_id is None or not self.mission_template_ids or mission_template_id in self.mission_template_ids)
        )
        ####

    def build(self) -> tuple[StandardFamilyAdapter, TuningCampaign]:
        """Construct and cross-check the plug-in-owned adapter and campaign."""

        adapter = self.adapter_factory()
        campaign = self.campaign_factory()
        descriptor = adapter.describe()
        if descriptor.family_id != self.family_id:
            raise ValueError(f"tuning registration {self.id!r} built adapter family {descriptor.family_id!r}, expected {self.family_id!r}")
        if descriptor.tier != self.fidelity:
            raise ValueError(f"tuning registration {self.id!r} built adapter fidelity {descriptor.tier!r}, expected {self.fidelity!r}")
        if campaign.family_id != self.family_id or campaign.tier != self.fidelity:
            raise ValueError(f"tuning registration {self.id!r} campaign identity does not match its declared family and fidelity")
        return adapter, campaign
        ####

    def adapter_advertisement(self) -> dict[str, Any]:
        """Describe the exact adapter used by this campaign without tuning it.

        A general family adapter may legitimately cover a different fidelity
        than a local reduced-order or response-law campaign.  This method
        exposes the campaign's own state/control schema and operation
        availability so planning tools can distinguish those two seams before
        they invoke trim, linearization, or a numerical candidate sweep.
        """

        adapter = self.adapter_factory()
        descriptor = adapter.describe()
        if descriptor.family_id != self.family_id or descriptor.tier != self.fidelity:
            raise ValueError(f"tuning registration {self.id!r} adapter advertisement does not match {self.family_id!r}/{self.fidelity!r}")
        capabilities = adapter.capability_report()
        return {
            "status": "available",
            "descriptor": descriptor.as_dict(),
            "operations": [item.as_dict() for item in capabilities.capabilities],
            "claim_boundary": (
                "This is the campaign-owned adapter descriptor and operation advertisement. It does not execute trim, linearization, or controller synthesis."
            ),
        }
        ####

    def local_controller_screen_advertisements(self, mission_template_id: str | None) -> tuple[dict[str, Any], ...]:
        """Return selected static screen records without running a campaign."""

        if mission_template_id is None:
            return ()
        return tuple(
            deepcopy(dict(screen))
            for screen in self.local_controller_screens
            if screen["mission_template_id"] == mission_template_id
        )
        ####

    def run(self) -> TuningCampaignReport:
        """Execute the declared inputs through the common campaign runner."""

        from .tuning_campaign import run_tuning_campaign

        adapter, campaign = self.build()
        return run_tuning_campaign(adapter, campaign)
        ####

    def run_cached(
        self,
        cache_dir: str | Path | None,
        *,
        context_fingerprint: str = "",
    ) -> CachedTuningCampaignResult:
        """Run once per exact declaration/adapter/context fingerprint."""

        from .tuning_campaign import run_tuning_campaign

        adapter, campaign = self.build()
        key_payload = {
            "cache_contract": "taoryx.tuning-campaign-resolved-gains/v2alpha1",
            "registration": self.public_dict(),
            "adapter": adapter.describe().as_dict(),
            "campaign": campaign.as_dict(),
            "context_fingerprint": context_fingerprint,
        }
        encoded = json.dumps(key_payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        cache_key = hashlib.sha256(encoded).hexdigest()
        if cache_dir is None:
            report = run_tuning_campaign(adapter, campaign)
            return CachedTuningCampaignResult(report.as_dict(), cache_key, False, None)

        directory = Path(cache_dir)
        cache_path = directory / f"{self.id}-{cache_key}.json"
        if cache_path.is_file():
            payload = json.loads(cache_path.read_text(encoding="utf-8"))
            if (
                isinstance(payload, dict)
                and payload.get("schema") == "taoryx.tuning-campaign/v1alpha1"
                and isinstance(payload.get("campaign"), dict)
                and payload["campaign"].get("campaign_id") == self.id
            ):
                return CachedTuningCampaignResult(payload, cache_key, True, cache_path)

        report = run_tuning_campaign(adapter, campaign)
        payload = report.as_dict()
        directory.mkdir(parents=True, exist_ok=True)
        temporary = cache_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(cache_path)
        return CachedTuningCampaignResult(payload, cache_key, False, cache_path)
        ####

    def application_contexts(
        self,
        cached: CachedTuningCampaignResult,
    ) -> tuple["TuningApplicationContext", ...]:
        """Project a candidate-ready cache payload into runtime-safe contexts.

        The shared host owns this projection so plug-ins do not each invent a
        partial interpretation of campaign JSON.  A context still does not
        instantiate a controller: the source-owning batch factory must check
        coordinate compatibility, apply the resolved gains, and then emit the
        context's runtime binding receipt.
        """

        from .tuning_application import tuning_application_contexts_from_campaign_payload

        return tuning_application_contexts_from_campaign_payload(
            cached.payload,
            campaign_id=self.id,
            cache_key=cached.cache_key,
            cache_hit=cached.cache_hit,
            cache_persisted=cached.cache_path is not None,
        )
        ####

    def public_dict(self) -> dict[str, Any]:
        """Return discovery metadata without constructing an adapter or campaign."""

        return {
            "id": self.id,
            "provider_id": self.provider_id,
            "provider_aliases": list(self.provider_aliases),
            "model_id": self.model_id,
            "family_id": self.family_id,
            "fidelity": self.fidelity,
            "realization_ids": list(self.realization_ids),
            "mission_template_ids": list(self.mission_template_ids),
            "description": self.description,
            "local_controller_screens": [deepcopy(dict(item)) for item in self.local_controller_screens],
            "tuning_application_context": {
                "schema": "taoryx.tuning-application-context/v1alpha1",
                "status": "available_when_candidate_ready",
                "supported_methods": ["lqr", "lqi"],
                "requires_exact_coordinate_match": True,
                "runtime_binding_fields": [
                    "campaign_id",
                    "node_id",
                    "candidate_profile_id",
                    "candidate_configuration_fingerprint_sha256",
                    "applied_gain_fingerprint_sha256",
                    "controller_method",
                    "cache_key",
                ],
                "claim_boundary": (
                    "The host can project selected campaign candidates into typed application contexts. A plug-in must "
                    "still instantiate the resolved gains in coordinate-compatible runtime controls before emitting a "
                    "tuning binding."
                ),
            },
            "required_extras": list(self.required_extras),
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


class ControllerTuningCampaignRegistry:
    """Immutable catalog of plug-in-contributed controller campaigns."""

    def __init__(
        self,
        registrations: Sequence[ControllerTuningCampaignRegistration] = (),
    ) -> None:
        self._registrations = tuple(registrations)
        if any(not isinstance(item, ControllerTuningCampaignRegistration) for item in self._registrations):
            raise TypeError("controller-tuning registry accepts only typed registrations")
        identifiers = tuple(item.id for item in self._registrations)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("controller-tuning registry contains duplicate campaign IDs")
        self._by_id = {item.id: item for item in self._registrations}
        ####

    @property
    def registrations(self) -> tuple[ControllerTuningCampaignRegistration, ...]:
        """Return registrations in deterministic plug-in declaration order."""

        return self._registrations
        ####

    def registration(self, identifier: str) -> ControllerTuningCampaignRegistration:
        """Resolve one exact campaign identity."""

        try:
            return self._by_id[identifier]
        except KeyError as error:
            raise KeyError(f"unknown controller-tuning campaign {identifier!r}") from error
        ####

    def matching(
        self,
        *,
        provider_id: str,
        model_id: str,
        fidelity: str,
        realization_id: str | None = None,
        mission_template_id: str | None = None,
    ) -> tuple[ControllerTuningCampaignRegistration, ...]:
        """Return campaigns applicable to one advertised model selection."""

        return tuple(
            item
            for item in self._registrations
            if item.matches(
                provider_id=provider_id,
                model_id=model_id,
                fidelity=fidelity,
                realization_id=realization_id,
                mission_template_id=mission_template_id,
            )
        )
        ####

    def public_dict(self) -> dict[str, object]:
        """Return the complete non-executable campaign advertisement."""

        return {
            "schema": "taoryx.controller-tuning-campaign-catalog/v1",
            "campaigns": [item.public_dict() for item in self._registrations],
            "claim_boundary": (
                "Campaign registration means the plug-in supplies declared inputs to the shared design screen; it is not controller or mission qualification."
            ),
        }
        ####

    def validate_against(
        self,
        providers: ConfigurableTrajectoryProviderRegistry,
    ) -> None:
        """Fail if a campaign drifts from its provider's portable advertisement."""

        for registration in self._registrations:
            model = providers.model(registration.provider_id, registration.model_id)
            if model.family_id != registration.family_id:
                raise ValueError(
                    f"controller-tuning campaign {registration.id!r} declares family {registration.family_id!r}, advertised model uses {model.family_id!r}"
                )
            fidelities = {item.id for item in model.fidelities if item.declared}
            if registration.fidelity not in fidelities:
                raise ValueError(f"controller-tuning campaign {registration.id!r} references unadvertised fidelity {registration.fidelity!r}")
            realizations = {item.id: item for item in model.realizations}
            unknown_realizations = sorted(set(registration.realization_ids) - set(realizations))
            if unknown_realizations:
                raise ValueError(f"controller-tuning campaign {registration.id!r} references unknown realizations {unknown_realizations!r}")
            incompatible_realizations = sorted(
                identifier for identifier in registration.realization_ids if registration.fidelity not in realizations[identifier].fidelity_aliases
            )
            if incompatible_realizations:
                raise ValueError(f"controller-tuning campaign {registration.id!r} has fidelity-incompatible realizations {incompatible_realizations!r}")
            missions = {item.id: item for item in model.mission_templates}
            unknown_missions = sorted(set(registration.mission_template_ids) - set(missions))
            if unknown_missions:
                raise ValueError(f"controller-tuning campaign {registration.id!r} references unknown missions {unknown_missions!r}")
            incompatible_missions = sorted(
                identifier for identifier in registration.mission_template_ids if registration.fidelity not in missions[identifier].compatible_fidelities
            )
            if incompatible_missions:
                raise ValueError(f"controller-tuning campaign {registration.id!r} has fidelity-incompatible missions {incompatible_missions!r}")
        ####

    ####


__all__ = [
    "CachedTuningCampaignResult",
    "ControllerTuningAdapterFactory",
    "ControllerTuningCampaignFactory",
    "ControllerTuningCampaignRegistration",
    "ControllerTuningCampaignRegistry",
]
####
