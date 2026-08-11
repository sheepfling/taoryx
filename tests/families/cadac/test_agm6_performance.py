from __future__ import annotations

from pathlib import Path
from time import perf_counter

from taoryx.families.cadac.agm6_mission_composition import (
    CadacAgm6MissionCompositionProvider,
    build_default_agm6_configuration,
)
from taoryx.families.cadac.agm6_plugin import Agm6VehiclePlugin
from test_agm6 import _write_agm6_case

from taoryx.trajectory.mission_composition import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionStepRequest,
)


def test_agm6_persistent_source_loop_stays_within_interactive_smoke_budget(tmp_path: Path) -> None:
    """Catch batch replay or per-step three-actor/sensor reconstruction."""

    provider = CadacAgm6MissionCompositionProvider(Agm6VehiclePlugin(_write_agm6_case(tmp_path)))
    configuration = build_default_agm6_configuration(
        provider,
        overrides={
            "end_time_s": 0.25,
            "sample_step_s": 0.01,
            "missile_position_ned_m": (0.0, 0.0, -10_000.0),
            "missile_pitch_deg": 45.0,
            "target_position_ned_m": (0.0, 100_000.0, -10_000.0),
            "aircraft_position_ned_m": (0.0, -10_000.0, -10_000.0),
            "navigation_gain": 0.0,
        },
    )
    prepared = provider.validate_configuration(configuration)
    provider.open_session(
        MissionCompositionOpenSessionRequest(
            session_id="agm6-performance",
            provider_id="cadac",
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.001,
        )
    )
    started = perf_counter()
    for _ in range(250):
        provider.step_session(MissionCompositionSessionStepRequest(session_id="agm6-performance", duration_s=0.001))
    elapsed_s = perf_counter() - started

    assert provider.inspect_session("agm6-performance").lifecycle == "completed"
    assert len(provider.session_sensor_packets("agm6-performance")) == 251
    assert elapsed_s < 2.0


####
