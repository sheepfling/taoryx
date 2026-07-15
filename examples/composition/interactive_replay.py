"""Run a small deterministic interactive steering and replay session."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.runtime import (  # noqa: E402
    ControlSpec,
    EventAction,
    EventSpec,
    InteractiveSession,
    OutputSubscription,
    RuntimeProblem,
    RuntimeState,
    RuntimeVehicle,
    StatusSpec,
)
from taoryx.visualization import render_run_artifact_html  # noqa: E402


def _problem() -> RuntimeProblem:
    def derivative(state: RuntimeState) -> tuple[float, float]:
        return state.named.get("throttle", 0.0), state.named.get("fin_pitch", 0.0)

    vehicle = RuntimeVehicle(
        "player",
        RuntimeState(0.0, (0.0, 0.0), named={"downrange": 0.0, "pitch": 0.0}, value_names=("downrange", "pitch")),
        derivative=derivative,
        step_size=0.1,
    )
    return RuntimeProblem({vehicle.name: vehicle})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("build/composition/interactive"))
    arguments = parser.parse_args()
    session = InteractiveSession(
        _problem(),
        controls=(
            ControlSpec("throttle", unit="fraction", lower=0.0, upper=1.0),
            ControlSpec("fin_pitch", unit="normalized", lower=-1.0, upper=1.0, slew_rate=2.0),
        ),
        status_specs=(StatusSpec("downrange", source="downrange", unit="m"),),
        event_specs=(EventSpec("range-check", lambda state: state.named.get("downrange", 0.0) >= 0.1, EventAction.SIGNAL),),
        output_subscriptions=(OutputSubscription(("downrange", "pitch"), sample_interval=0.1),),
    )
    session.step(0.1, {"throttle": 0.8, "fin_pitch": 0.5})
    session.step(0.1, {"throttle": 0.8, "fin_pitch": -0.5})
    artifact = session.to_run_artifact()
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    artifact.write_json(arguments.output_dir / "interactive.json")
    artifact.write_csv(arguments.output_dir / "interactive.csv", vehicle_id="player")
    render_run_artifact_html(artifact, arguments.output_dir / "interactive.html", vehicle_id="player")
    print(f"artifacts={arguments.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
