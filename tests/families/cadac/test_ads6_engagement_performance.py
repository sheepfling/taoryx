from __future__ import annotations

from pathlib import Path
from time import perf_counter

from taoryx.families.cadac.ads6_engagement_mission_composition import (
    ADS6_ENGAGEMENT_MODEL_VERSION,
    CadacAds6EngagementMissionCompositionProvider,
    build_default_ads6_engagement_configuration,
)
from taoryx.families.cadac.ads6_engagement_plugin import Ads6EngagementPlugin
from test_ads6_engagement import _write_source_controller_aircraft_engagement

from taoryx.trajectory.mission_composition import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionStepRequest,
)


def test_ads6_engagement_persistent_package_loop_stays_within_interactive_smoke_budget(tmp_path: Path) -> None:
    """Catch batch replay or per-step package/controller/sensor reconstruction."""

    provider = CadacAds6EngagementMissionCompositionProvider(
        Ads6EngagementPlugin(_write_source_controller_aircraft_engagement(tmp_path))
    )
    configuration = build_default_ads6_engagement_configuration(
        provider,
        overrides={"end_time_s": 2.5, "sample_step_s": 0.01},
    )
    prepared = provider.validate_configuration(configuration)
    provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="ads6-engagement-performance",
            provider_id="cadac",
            provider_version=ADS6_ENGAGEMENT_MODEL_VERSION,
            prepared_configuration=prepared,
            integration_step_s=0.01,
        )
    )
    started = perf_counter()
    for _ in range(251):
        provider.step_session(
            MissionCompositionSessionStepRequest(session_id="ads6-engagement-performance", duration_s=0.01)
        )
    elapsed_s = perf_counter() - started

    assert provider.inspect_session("ads6-engagement-performance").lifecycle == "completed"
    assert len(provider.session_sensor_packets("ads6-engagement-performance")) == 252
    assert elapsed_s < 2.0


####
