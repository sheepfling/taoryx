"""Executable ADS6 package-composition plug-in bound to one installed source case."""

from __future__ import annotations

from pathlib import Path

from .ads6_engagement import (
    Ads6EngagementRunConfig,
    Ads6EngagementRunResult,
    Ads6EngagementSourceDefinition,
    load_ads6_engagement_source_definition,
    run_ads6_engagement,
)


class Ads6EngagementPlugin:
    """Installed source-ordered ADS6 SAM/target/RADAR0 package plug-in."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: Ads6EngagementSourceDefinition | None = None

    ####

    @property
    def source_case_path(self) -> Path:
        return self._source_case_path

    ####

    def validate_installation(self) -> tuple[str, ...]:
        if not self._source_case_path.is_file():
            return (f"source case does not exist: {self._source_case_path}",)
        ####
        try:
            definition = self._load_definition()
        except (OSError, ValueError) as error:
            return (f"source package could not be lowered: {error}",)
        ####
        blockers: list[str] = []
        if not 1 <= len(definition.sams) <= 3:
            blockers.append("ADS6 package runtime requires one to three SAM actors")
        ####
        if definition.radar.track_mode != definition.target_kind:
            blockers.append("ADS6 radar tracking mode must match package target type")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Ads6EngagementSourceDefinition:
        blockers = self.validate_installation()
        if blockers:
            raise ValueError("ADS6 package plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, config: Ads6EngagementRunConfig | None = None) -> Ads6EngagementRunResult:
        return run_ads6_engagement(self.source_definition(), config)

    ####

    def _load_definition(self) -> Ads6EngagementSourceDefinition:
        if self._definition is None:
            self._definition = load_ads6_engagement_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


__all__ = ["Ads6EngagementPlugin"]
