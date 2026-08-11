"""Run the three deterministic Mission Composition consumer fixtures."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import cast

from taoryx.model_authoring import run_prepared_mission_composition
from taoryx.plugins import discover_plugins
from taoryx.trajectory.configuration_contract import PreparedTrajectoryConfiguration
from taoryx.trajectory.contract_probe_mission_composition import (
    ContractProbeMissionCompositionProvider,
    build_contract_probe_configuration,
)
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection
from taoryx.trajectory.mission_composition import (
    ReferenceBallisticLaunch,
    ReferenceMissionCompositionProvider,
    ReferenceWaypoint,
    ReferenceWaypointCourseStart,
)


def _ballistic(provider: ReferenceMissionCompositionProvider) -> PreparedTrajectoryConfiguration:
    """Build the smallest repeatable ballistic coast composition."""

    return provider.prepare_ballistic(
        ReferenceBallisticLaunch(
            altitude_m=1000.0,
            speed_m_s=150.0,
            heading_deg=0.0,
            flight_path_angle_deg=20.0,
        ),
        coast_durations_s=(2.0, 2.0),
        configuration_id="consumer-ballistic",
    )
    ####


def _waypoint(provider: ReferenceMissionCompositionProvider) -> PreparedTrajectoryConfiguration:
    """Build a two-leg outbound/return constant-velocity waypoint route."""

    return provider.prepare_waypoint_course(
        ReferenceWaypointCourseStart(
            altitude_m=1000.0,
            speed_m_s=100.0,
            heading_deg=90.0,
        ),
        waypoints=(
            ReferenceWaypoint(
                instance_id="outbound",
                north_m=0.0,
                east_m=500.0,
                altitude_m=1000.0,
            ),
            ReferenceWaypoint(
                instance_id="return",
                north_m=0.0,
                east_m=0.0,
                altitude_m=1000.0,
            ),
        ),
        configuration_id="consumer-waypoint",
    )
    ####


def main() -> int:
    """Compose, revalidate, execute, and serialize all three fixtures."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write the full discriminated responses as JSON")
    args = parser.parse_args()

    providers = discover_plugins().build_mission_composition_provider_registry()
    reference = cast(
        ReferenceMissionCompositionProvider,
        providers.provider("taoryx.reference.mission-composition"),
    )
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
