"""Typed navigation-to-controller feedback contracts."""

from __future__ import annotations

from typing import Literal, Mapping

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, model_validator


class NavigationFeedbackConfig(BaseModel):
    """Select which state a controller may consume after sensor delivery."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    source: Literal["plant-truth", "dead-reckoning", "mekf"] = "plant-truth"
    availability: Literal["delivered"] = "delivered"
    stale_policy: Literal["hold", "fail"] = "hold"
    max_age_s: float | None = Field(default=None, ge=0.0)

    @model_validator(mode="after")
    def validate_stale_policy(self) -> NavigationFeedbackConfig:
        if self.stale_policy == "fail" and self.max_age_s is None:
            raise ValueError("feedback max_age_s is required when stale_policy is fail")
        return self


_FEEDBACK_ADAPTER = TypeAdapter(NavigationFeedbackConfig)


def parse_navigation_feedback(value: Mapping[str, object] | None) -> NavigationFeedbackConfig:
    """Parse an optional controller feedback declaration."""

    return _FEEDBACK_ADAPTER.validate_python(dict(value or {}))
