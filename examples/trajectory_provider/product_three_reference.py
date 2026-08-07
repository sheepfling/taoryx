"""Run the checked-in Product 3 provider example.

This is intentionally a normal Python plug-in flow: discover metadata,
prepare a typed request, execute it, and write the standard trajectory
envelope.  The provider itself is analytical reference code; it is not a
source-grounded vehicle model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.trajectory import (
    ExampleProductThreeProvider,
    ProductThreeOutputRequest,
    ProductThreeParameterValue,
    ProductThreeProviderRegistry,
    ProductThreeSegmentRequest,
    ProductThreeTrajectoryRequest,
)


def example_request() -> ProductThreeTrajectoryRequest:
    """Return a two-waypoint route followed by a bounded bank maneuver."""

    return ProductThreeTrajectoryRequest(
        request_id="product-three-waypoint-demo",
        vehicle_id="reference_guided_point_mass",
        fidelity="point_mass_3dof",
        initialization_id="airborne_state",
        initialization={
            "altitude_m": ProductThreeParameterValue(value=1000.0, unit="m"),
            "speed_m_s": ProductThreeParameterValue(value=100.0, unit="m/s"),
            "heading_deg": ProductThreeParameterValue(value=90.0, unit="deg"),
        },
        segments=(
            ProductThreeSegmentRequest(
                id="waypoint_leg",
                instance_id="outbound",
                parameters={
                    "duration_s": ProductThreeParameterValue(value=5.0, unit="s"),
                    "waypoint_north_m": ProductThreeParameterValue(value=0.0, unit="m"),
                    "waypoint_east_m": ProductThreeParameterValue(value=500.0, unit="m"),
                    "waypoint_altitude_m": ProductThreeParameterValue(value=1000.0, unit="m"),
                    "max_load_factor_g": ProductThreeParameterValue(value=3.0, unit="g0"),
                },
            ),
            ProductThreeSegmentRequest(
                id="waypoint_leg",
                instance_id="return",
                parameters={
                    "duration_s": ProductThreeParameterValue(value=5.0, unit="s"),
                    "waypoint_north_m": ProductThreeParameterValue(value=0.0, unit="m"),
                    "waypoint_east_m": ProductThreeParameterValue(value=0.0, unit="m"),
                    "waypoint_altitude_m": ProductThreeParameterValue(value=1000.0, unit="m"),
                    "max_load_factor_g": ProductThreeParameterValue(value=2.5, unit="g0"),
                },
            ),
            ProductThreeSegmentRequest(
                id="bank_maneuver",
                instance_id="finish-heading",
                parameters={
                    "duration_s": ProductThreeParameterValue(value=2.0, unit="s"),
                    "target_heading_deg": ProductThreeParameterValue(value=90.0, unit="deg"),
                    "max_load_factor_g": ProductThreeParameterValue(value=2.0, unit="g0"),
                },
            ),
        ),
        output=ProductThreeOutputRequest(cadence_s=0.5),
    )
    ####


def main() -> int:
    """Print discovery and run the example request."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write the standard trajectory JSON to this path")
    args = parser.parse_args()

    provider = ExampleProductThreeProvider()
    registry = ProductThreeProviderRegistry((provider,))
    prepared = provider.prepare(example_request())
    result = provider.run(prepared)

    print(json.dumps(registry.catalog(), indent=2, sort_keys=True))
    print(json.dumps({"status": result.status, "samples": len(result.samples), "fingerprint": result.request_fingerprint}, indent=2))
    if args.output is not None:
        result.write_json(args.output)
        print(f"wrote {args.output}")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
