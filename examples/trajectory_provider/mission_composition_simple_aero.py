"""Author and run one compact Simple Aero mission using every typed segment form."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.model_authoring import run_prepared_mission_composition
from taoryx.plugins import discover_plugins
from taoryx.trajectory import (
    MissionCompositionOutputSelection,
    SimpleAeroCheckpoints,
    SimpleAeroEndpointState,
    SimpleAeroGeodeticAimpoint,
    SimpleAeroLaunch,
    SimpleAeroMission,
    SimpleAeroRuntime,
    SimpleAeroSegment,
    SimpleAeroSurrogate,
    prepare_simple_aero_mission,
)


def example_mission() -> SimpleAeroMission:
    """Return a short custom sequence that covers the advertised source vocabulary."""

    return SimpleAeroMission(
        configuration_id="simple-aero-ergonomic-feature-space",
        launch=SimpleAeroLaunch(
            latitude_deg=35.8766,
            longitude_deg=14.4425,
            altitude_m=500.0,
            speed_m_s=75.0,
            pitch_over_angle_deg=20.0,
            initial_heading_offset_deg=10.0,
        ),
        endpoint=SimpleAeroGeodeticAimpoint(latitude_deg=36.000975, longitude_deg=-5.60999),
        endpoint_state=SimpleAeroEndpointState(altitude_m=200.0, speed_m_s=50.0, heading_deg=20.0),
        surrogate=SimpleAeroSurrogate(
            vehicle_id="simple-aero-feature-demo",
            initial_mass_kg=1200.0,
            thrust_n=90_000.0,
            mass_flow_kg_s=15.0,
            drag_coefficient=0.03,
            lift_to_drag=3.5,
        ),
        checkpoints=SimpleAeroCheckpoints(burnout_speed_m_s=900.0, apogee_altitude_m=20_000.0),
        runtime=SimpleAeroRuntime(time_step_s=0.05, output_interval_s=0.1, earth_model="vacuum_spherical"),
        segments=(
            SimpleAeroSegment.powered_ascent(duration_s=0.2, cutoff_condition="commanded_burnout_speed"),
            SimpleAeroSegment.ballistic_coast(duration_s=0.2, alpha_deg=2.0),
            SimpleAeroSegment.bank_maneuver(duration_s=0.2, bank_deg=-12.0),
            SimpleAeroSegment.cbcr(
                go_left=False,
                maneuver_altitude_start_m=10_000.0,
                duration_s=0.2,
                minimum_time_to_go_s=0.2,
            ),
            SimpleAeroSegment.crossrange(initial_heading_error_deg=8.0, minimum_time_to_go_s=0.2),
            SimpleAeroSegment.marv(maneuver_begin_time_to_go_s=0.2, minimum_time_to_go_s=0.2),
            SimpleAeroSegment.phugoid(
                start_range_to_go_m=100.0,
                amplitude_deg=1.0,
                frequency_hz=2.0,
                maneuver_roll_deg=20.0,
            ),
            SimpleAeroSegment.range_extension(minimum_time_to_go_s=0.2),
            SimpleAeroSegment.skip(maneuver_begin_time_to_go_s=0.2),
            SimpleAeroSegment.slalom(
                start_range_to_go_m=100.0,
                end_range_to_go_m=50.0,
                minimum_time_to_go_s=0.2,
            ),
            SimpleAeroSegment.weave(end_range_to_go_m=25.0, minimum_time_to_go_s=0.2),
            SimpleAeroSegment.terminal_pronav(duration_s=0.2, capture_range_m=5.0),
        ),
    )
    ####


def main() -> int:
    """Prepare, revalidate, run, and optionally serialize the full feature witness."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write the discriminated common response as JSON")
    args = parser.parse_args()

    providers = discover_plugins().build_mission_composition_provider_registry()
    provider = providers.provider("taoryx.registry.mission-composition")
    prepared = provider.validate_configuration(prepare_simple_aero_mission(example_mission()).configuration)
    response = run_prepared_mission_composition(
        providers,
        provider.metadata.id,
        prepared,
        output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=7),
    )
    payload = response.model_dump(mode="json", by_alias=True)
    summary = {
        "response_kind": response.kind,
        "model_id": response.result.primary_model_id if response.kind == "trajectory" else response.failure.model_id,
        "segment_count": len(prepared.resolved["segments"]),
        "operation_template_id": prepared.configuration.mission_template_id,
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"wrote {args.output}")
    return 0 if response.kind == "trajectory" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
