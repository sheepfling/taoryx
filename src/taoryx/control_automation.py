"""Compact plug-in declarations for automatic local controller campaigns."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Literal

from .generic_tuning import LinearAuthorityRequirement, NormalizedLqrProfileGrid
from .tuning_campaign import TuningCampaign, TuningCampaignNode

AutomaticControllerMethod = Literal["auto", "lqr", "lqi"]


@dataclass(frozen=True, slots=True)
class ControlAutomationDeclaration:
    """Minimal model-owned data needed to generate a common tuning campaign.

    The plug-in names the local plant coordinates, engineering scales, trim
    inputs, and outputs that require offset-free tracking. Taoryx owns method
    selection, normalized candidate generation, authority preflight, and the
    standard stop-at-first-blocker workflow.
    """

    id: str
    campaign_id: str
    family_id: str
    tier: str
    strategy_id: str
    node_id: str
    state_scales: Mapping[str, float]
    control_scales: Mapping[str, float]
    trim_target: Mapping[str, float] = field(default_factory=dict)
    trim_initial_guess: Mapping[str, float] = field(default_factory=dict)
    design_state_names: tuple[str, ...] = ()
    design_control_names: tuple[str, ...] = ()
    authority_state_names: tuple[str, ...] = ()
    offset_free_outputs: tuple[str, ...] = ()
    preferred_method: AutomaticControllerMethod = "auto"
    profile_grid_id_prefix: str | None = None
    state_weight_multipliers: tuple[float, ...] = (0.25, 1.0, 4.0)
    control_effort_multipliers: tuple[float, ...] = (2.0, 1.0, 0.25)
    state_base_weights: tuple[float, ...] = ()
    control_base_weights: tuple[float, ...] = ()
    integral_weight_multiplier: float = 8.0
    integral_base_weights: tuple[float, ...] = ()
    integral_weight_multipliers: tuple[float, ...] = (1.0,)
    maximum_omitted_state_coupling: float = 1.0e-8
    linearization_options: Mapping[str, float | str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        identities = (self.id, self.campaign_id, self.family_id, self.tier, self.strategy_id, self.node_id)
        if not all(value.strip() for value in identities):
            raise ValueError("control-automation declarations require non-empty identities")
        if not self.state_scales or not self.control_scales:
            raise ValueError("control automation requires state and control scales")
        for label, values in (("state", self.state_scales), ("control", self.control_scales)):
            if any(not name.strip() for name in values):
                raise ValueError(f"control-automation {label} names must not be empty")
            if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in values.values()):
                raise ValueError(f"control-automation {label} scales must be finite and positive")
        states = tuple(self.state_scales)
        controls = tuple(self.control_scales)
        design_states = self.design_state_names or states
        design_controls = self.design_control_names or controls
        if len(set(design_states)) != len(design_states) or set(design_states) - set(states):
            raise ValueError("control-automation design states must be unique declared state scales")
        if len(set(design_controls)) != len(design_controls) or set(design_controls) - set(controls):
            raise ValueError("control-automation design controls must be unique declared control scales")
        authority = self.authority_state_names or design_states
        if len(set(authority)) != len(authority) or set(authority) - set(design_states):
            raise ValueError("control-automation authority states must be unique design states")
        if len(set(self.offset_free_outputs)) != len(self.offset_free_outputs) or set(self.offset_free_outputs) - set(design_states):
            raise ValueError("control-automation offset-free outputs must be unique design states")
        if self.preferred_method == "lqr" and self.offset_free_outputs:
            raise ValueError("an LQR-only declaration cannot request offset-free outputs")
        if self.preferred_method == "lqi" and not self.offset_free_outputs:
            raise ValueError("an LQI declaration requires offset-free outputs")
        if self.profile_grid_id_prefix is not None and not self.profile_grid_id_prefix.strip():
            raise ValueError("control-automation profile-grid ID prefixes must not be empty")
        for label, weight_values, expected_count in (
            ("state weight multipliers", self.state_weight_multipliers, None),
            ("control effort multipliers", self.control_effort_multipliers, None),
            ("state base weights", self.state_base_weights, len(design_states)),
            ("control base weights", self.control_base_weights, len(design_controls)),
        ):
            if not weight_values:
                if expected_count is not None:
                    continue
                raise ValueError(f"control-automation {label} must not be empty")
            if any(not math.isfinite(value) or value <= 0.0 for value in weight_values):
                raise ValueError(f"control-automation {label} must be finite and positive")
            if len(weight_values) != len(set(weight_values)) and expected_count is None:
                raise ValueError(f"control-automation {label} must not contain duplicates")
            if expected_count is not None and len(weight_values) != expected_count:
                raise ValueError(f"control-automation {label} must match the selected coordinate count")
        if not math.isfinite(self.integral_weight_multiplier) or self.integral_weight_multiplier <= 0.0:
            raise ValueError("control-automation integral weight multiplier must be finite and positive")
        if self.integral_base_weights:
            if self.controller_method != "lqi":
                raise ValueError("LQR-only control automation cannot declare integral base weights")
            if len(self.integral_base_weights) != len(self.offset_free_outputs):
                raise ValueError("control-automation integral base weights must match the selected LQI outputs")
            if any(not math.isfinite(value) or value <= 0.0 for value in self.integral_base_weights):
                raise ValueError("control-automation integral base weights must be finite and positive")
        if (
            not self.integral_weight_multipliers
            or any(not math.isfinite(value) or value <= 0.0 for value in self.integral_weight_multipliers)
            or len(self.integral_weight_multipliers) != len(set(self.integral_weight_multipliers))
        ):
            raise ValueError("control-automation integral weight multipliers must be unique finite positive values")
        if self.controller_method == "lqr" and self.integral_weight_multipliers != (1.0,):
            raise ValueError("LQR-only control automation cannot vary integral weights")
        if not math.isfinite(self.maximum_omitted_state_coupling) or self.maximum_omitted_state_coupling < 0.0:
            raise ValueError("control-automation omitted-state coupling limit must be finite and nonnegative")
        ####

    @property
    def controller_method(self) -> Literal["lqr", "lqi"]:
        """Resolve the conservative common method from declared intent."""

        if self.preferred_method != "auto":
            return self.preferred_method
        return "lqi" if self.offset_free_outputs else "lqr"
        ####

    def build_campaign(self) -> TuningCampaign:
        """Generate the standard campaign without model-specific tuning code."""

        states = self.design_state_names or tuple(self.state_scales)
        controls = self.design_control_names or tuple(self.control_scales)
        authority_states = self.authority_state_names or states
        method = self.controller_method
        return TuningCampaign(
            campaign_id=self.campaign_id,
            family_id=self.family_id,
            tier=self.tier,
            strategy_id=self.strategy_id,
            nodes=(
                TuningCampaignNode(
                    node_id=self.node_id,
                    trim_target=dict(self.trim_target),
                    trim_initial_guess=dict(self.trim_initial_guess),
                    state_scales=tuple(float(self.state_scales[name]) for name in states),
                    control_scales=tuple(float(self.control_scales[name]) for name in controls),
                    authority_requirement=LinearAuthorityRequirement(
                        f"{self.id}.authority",
                        tuple(authority_states),
                    ),
                    profile_grid=NormalizedLqrProfileGrid(
                        self.profile_grid_id_prefix or f"{self.id}.{method}",
                        state_weight_multipliers=self.state_weight_multipliers,
                        control_effort_multipliers=self.control_effort_multipliers,
                        integral_weight_multipliers=self.integral_weight_multipliers,
                        state_base_weights=self.state_base_weights,
                        control_base_weights=self.control_base_weights,
                    ),
                    design_state_names=states,
                    design_control_names=controls,
                    controller_method=method,
                    integral_output_names=self.offset_free_outputs if method == "lqi" else (),
                    integral_q_diagonal=(
                        self.integral_base_weights
                        if method == "lqi" and self.integral_base_weights
                        else tuple(self.integral_weight_multiplier for _ in self.offset_free_outputs)
                        if method == "lqi"
                        else ()
                    ),
                    maximum_omitted_state_coupling=self.maximum_omitted_state_coupling,
                    linearization_options=dict(self.linearization_options),
                ),
            ),
        )
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the small, portable declaration supplied by a plug-in."""

        return {
            "id": self.id,
            "campaign_id": self.campaign_id,
            "family_id": self.family_id,
            "tier": self.tier,
            "strategy_id": self.strategy_id,
            "node_id": self.node_id,
            "state_scales": dict(self.state_scales),
            "control_scales": dict(self.control_scales),
            "trim_target": dict(self.trim_target),
            "trim_initial_guess": dict(self.trim_initial_guess),
            "design_state_names": list(self.design_state_names),
            "design_control_names": list(self.design_control_names),
            "authority_state_names": list(self.authority_state_names),
            "offset_free_outputs": list(self.offset_free_outputs),
            "preferred_method": self.preferred_method,
            "profile_grid_id_prefix": self.profile_grid_id_prefix,
            "state_weight_multipliers": list(self.state_weight_multipliers),
            "control_effort_multipliers": list(self.control_effort_multipliers),
            "state_base_weights": list(self.state_base_weights),
            "control_base_weights": list(self.control_base_weights),
            "resolved_method": self.controller_method,
            "integral_weight_multiplier": self.integral_weight_multiplier,
            "integral_base_weights": list(self.integral_base_weights),
            "integral_weight_multipliers": list(self.integral_weight_multipliers),
            "maximum_omitted_state_coupling": self.maximum_omitted_state_coupling,
            "linearization_options": dict(self.linearization_options),
        }
        ####


__all__ = ["AutomaticControllerMethod", "ControlAutomationDeclaration"]
####
