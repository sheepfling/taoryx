"""Run the three deterministic Mission Composition consumer fixtures."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import cast

from taoryx.model_authoring import (
    author_configuration,
    custom_sequence,
    run_prepared_mission_composition,
    segment_occurrence,
    select_variant,
)
from taoryx.plugins import discover_plugins
from taoryx.trajectory.contract_probe_mission_composition import (
    ContractProbeMissionCompositionProvider,
    build_contract_probe_configuration,
)
from taoryx.trajectory.configuration_contract import (
    ConfigurableTrajectoryProvider,
    PreparedTrajectoryConfiguration,
)
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection


def _ballistic(provider: ConfigurableTrajectoryProvider) -> PreparedTrajectoryConfiguration:
    """Build the smallest repeatable ballistic coast composition."""

    return author_configuration(
        provider,
        configuration_id="consumer-ballistic",
        model_id="reference_ballistic_3dof",
        fidelity="point_mass_3dof",
        realization_id="analytical_point_mass",
        values={
            "initialization": select_variant(
                "launch_state",
                altitude_m=1000.0,
                speed_m_s=150.0,
                heading_deg=0.0,
                flight_path_angle_deg=20.0,
            ),
            "segments": custom_sequence(
                segment_occurrence("ballistic_coast", duration_s=2.0),
                segment_occurrence("ballistic_coast", duration_s=2.0),
            ),
        },
    )
    ####


def _waypoint(provider: ConfigurableTrajectoryProvider) -> PreparedTrajectoryConfiguration:
    """Build a two-leg outbound/return constant-velocity waypoint route."""

    return author_configuration(
        provider,
        configuration_id="consumer-waypoint",
        model_id="reference_constant_velocity_waypoint_3dof",
        fidelity="point_mass_3dof",
        realization_id="analytical_point_mass",
        values={
            "initialization": select_variant(
                "initial_state",
                altitude_m=1000.0,
                speed_m_s=100.0,
                heading_deg=90.0,
            ),
            "segments": custom_sequence(
                segment_occurrence(
                    "waypoint_leg",
                    instance_id="outbound",
                    duration_s=5.0,
                    waypoint_north_m=0.0,
                    waypoint_east_m=500.0,
                    waypoint_altitude_m=1000.0,
                ),
                segment_occurrence(
                    "waypoint_leg",
                    instance_id="return",
                    duration_s=5.0,
                    waypoint_north_m=0.0,
                    waypoint_east_m=0.0,
                    waypoint_altitude_m=1000.0,
                ),
            ),
        },
    )
    ####


def main() -> int:
    """Compose, revalidate, execute, and serialize all three fixtures."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write the full discriminated responses as JSON")
    args = parser.parse_args()

    providers = discover_plugins().build_mission_composition_provider_registry()
    reference = providers.provider("taoryx.reference.mission-composition")
    probe = cast(
        ContractProbeMissionCompositionProvider,
        providers.provider("taoryx.debug.mission-composition-contract-probe"),
    )
    runs = {
        "ballistic": run_prepared_mission_composition(providers, reference.metadata.id, _ballistic(reference)),
        "waypoint": run_prepared_mission_composition(providers, reference.metadata.id, _waypoint(reference)),
        "contract_probe": run_prepared_mission_composition(
            providers,
            probe.metadata.id,
            probe.validate_configuration(build_contract_probe_configuration(probe)),
            output=MissionCompositionOutputSelection(mode="all"),
        ),
    }
    payload = {
        "schema": "taoryx.leading-composition-consumer-proof/v1",
        "responses": {identifier: response.model_dump(mode="json", by_alias=True) for identifier, response in runs.items()},
    }
    summary = {
        identifier: {
            "kind": response.kind,
            "model_id": response.result.primary_model_id if response.kind == "trajectory" else response.failure.model_id,
            "object_count": len(response.result.objects) if response.kind == "trajectory" else 0,
        }
        for identifier, response in runs.items()
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"wrote {args.output}")
    return 0 if all(response.kind == "trajectory" for response in runs.values()) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
