"""Run the common Mission Composition interface against the waypoint fixture."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.trajectory.mission_composition import (
    ConfigurationChoiceValue,
    ConfigurationGroupValue,
    ConfigurationParameterValue,
    ConfigurationSequenceValue,
    MissionCompositionFailureResponse,
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    ReferenceMissionCompositionProvider,
    TrajectoryConfigurationInstance,
    audit_provider_advertisement,
)


def _parameter(value: float, unit: str) -> ConfigurationParameterValue:
    return ConfigurationParameterValue(value=value, unit=unit)
    ####


def example_configuration(provider: ReferenceMissionCompositionProvider) -> TrajectoryConfigurationInstance:
    """Return a typed two-leg constant-velocity waypoint route."""

    model_id = "reference_constant_velocity_waypoint_3dof"
    schema = provider.get_model_schema(model_id)
    return TrajectoryConfigurationInstance(
        configuration_id="constant-velocity-waypoint-demo",
        model_id=model_id,
        model_version=schema.model_version,
        schema_fingerprint=schema.fingerprint,
        fidelity="point_mass_3dof",
        root=ConfigurationGroupValue(
            values={
                "initialization": ConfigurationChoiceValue(
                    selected="initial_state",
                    value=ConfigurationGroupValue(
                        values={
                            "altitude_m": _parameter(1000.0, "m"),
                            "speed_m_s": _parameter(100.0, "m/s"),
                            "heading_deg": _parameter(90.0, "deg"),
                        }
                    ),
                ),
                "segments": ConfigurationSequenceValue(
                    items=(
                        ConfigurationChoiceValue(
                            selected="waypoint_leg",
                            value=ConfigurationGroupValue(
                                values={
                                    "duration_s": _parameter(5.0, "s"),
                                    "waypoint_north_m": _parameter(0.0, "m"),
                                    "waypoint_east_m": _parameter(500.0, "m"),
                                    "waypoint_altitude_m": _parameter(1000.0, "m"),
                                }
                            ),
                        ),
                        ConfigurationChoiceValue(
                            selected="waypoint_leg",
                            value=ConfigurationGroupValue(
                                values={
                                    "duration_s": _parameter(5.0, "s"),
                                    "waypoint_north_m": _parameter(0.0, "m"),
                                    "waypoint_east_m": _parameter(0.0, "m"),
                                    "waypoint_altitude_m": _parameter(1000.0, "m"),
                                }
                            ),
                        ),
                    )
                ),
            }
        ),
    )
    ####


def main() -> int:
    """Audit discovery, validate configuration, and execute through the common runner."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write the common trajectory response JSON")
    args = parser.parse_args()

    provider = ReferenceMissionCompositionProvider()
    runner = provider.build_runner()
    audit = audit_provider_advertisement(provider, runner)
    prepared = provider.validate_configuration(example_configuration(provider))
    request = MissionCompositionRunRequest(
        request_id="constant-velocity-waypoint-demo",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(cadence_s=0.5),
    )
    response = runner.run(request)
    payload = response.model_dump(mode="json", by_alias=True)
    print(
        json.dumps(
            {
                "advertisement_audit": audit.status,
                "models": audit.validated_model_count,
                "response_kind": response.kind,
                "request_id": request.request_id,
            },
            indent=2,
            sort_keys=True,
        )
    )
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"wrote {args.output}")
    return 1 if isinstance(response, MissionCompositionFailureResponse) else 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
