from __future__ import annotations

from pathlib import Path
from time import perf_counter

from taoryx.families.cadac.sraam6_mission_composition import (
    CadacSraam6MissionCompositionProvider,
    build_default_sraam6_configuration,
)
from taoryx.families.cadac.sraam6_plugin import Sraam6VehiclePlugin
from test_sraam6 import _write_sraam6_case

from taoryx.trajectory.mission_composition import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionStepRequest,
)


def test_sraam6_persistent_source_loop_stays_within_interactive_smoke_budget(tmp_path: Path) -> None:
    """Catch batch replay or per-step target/seeker/sensor reconstruction."""

    provider = CadacSraam6MissionCompositionProvider(Sraam6VehiclePlugin(_write_sraam6_case(tmp_path)))
    configuration = build_default_sraam6_configuration(
        provider,
        overrides={
            "end_time_s": 2.0,
            "sample_step_s": 0.01,
            "missile_position_ned_m": (0.0, 0.0, -10_000.0),
            "missile_pitch_deg": 45.0,
            "target_position_ned_m": (0.0, 100_000.0, -10_000.0),
            "navigation_gain": 0.0,
        },
    )
    prepared = provider.validate_configuration(configuration)
    provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="sraam6-performance",
            provider_id="cadac",
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.01,
        )
    )
    started = perf_counter()
    for _ in range(200):
        provider.step_session(MissionCompositionSessionStepRequest(session_id="sraam6-performance", duration_s=0.01))
    elapsed_s = perf_counter() - started

    assert provider.inspect_session("sraam6-performance").lifecycle == "completed"
    assert len(provider.session_sensor_packets("sraam6-performance")) == 201
    assert elapsed_s < 2.0


####
