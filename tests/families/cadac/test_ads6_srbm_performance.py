from __future__ import annotations

from pathlib import Path
from time import perf_counter

from taoryx.families.cadac.ads6_srbm_mission_composition import (
    CadacAds6SrbmMissionCompositionProvider,
    build_default_ads6_srbm_configuration,
)
from taoryx.families.cadac.ads6_srbm_plugin import Ads6SrbmVehiclePlugin
from test_ads6_srbm import _write_ads6_srbm_case

from taoryx.trajectory.mission_composition import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionStepRequest,
)


def test_ads6_srbm_persistent_source_loop_stays_within_interactive_smoke_budget(tmp_path: Path) -> None:
    """Catch accidental batch replay or per-step source/sensor reconstruction."""

    provider = CadacAds6SrbmMissionCompositionProvider(Ads6SrbmVehiclePlugin(_write_ads6_srbm_case(tmp_path)))
    configuration = build_default_ads6_srbm_configuration(provider, overrides={"end_time_s": 2.5, "sample_step_s": 0.01})
    prepared = provider.validate_configuration(configuration)
    provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="ads6-srbm-performance",
            provider_id="cadac",
            provider_version="0.9.0",
            prepared_configuration=prepared,
            integration_step_s=0.01,
        )
    )
    started = perf_counter()
    for _ in range(250):
        provider.step_session(MissionCompositionSessionStepRequest(session_id="ads6-srbm-performance", duration_s=0.01))
    elapsed_s = perf_counter() - started

    assert provider.inspect_session("ads6-srbm-performance").lifecycle == "completed"
    assert len(provider.session_sensor_packets("ads6-srbm-performance")) == 251
    assert elapsed_s < 2.0


####
