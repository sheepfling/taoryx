"""Validate control availability across the three explicit runtime modes."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.contracts import Frame, FrameVector3, Vector3  # noqa: E402
from taoryx.modes import DynamicsMode, Kinematic6DofState  # noqa: E402
from taoryx.runtime import ControlSpec, InteractiveSession, RuntimeProblem, RuntimeState, RuntimeVehicle  # noqa: E402


def _vehicle(name: str, mode: DynamicsMode) -> RuntimeVehicle:
    kwargs: dict[str, object] = {"dynamics_mode": mode, "derivative": lambda state: (0.0,)}
    if mode is DynamicsMode.KINEMATIC_6DOF:
        kwargs["kinematic_state"] = Kinematic6DofState(
            0.0,
            FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
            FrameVector3(Vector3(100.0, 0.0, 0.0), Frame.ECFC),
        )
        kwargs["body_rate_provider"] = lambda state: Vector3(0.0, 0.0, 0.0)
    return RuntimeVehicle(name, RuntimeState(0.0, (0.0,)), **kwargs)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    for mode in DynamicsMode:
        vehicle = _vehicle(mode.value, mode)
        InteractiveSession(RuntimeProblem({vehicle.name: vehicle}), controls=(ControlSpec("command", modes=(mode.value,)),))
        rejected = False
        wrong_mode = next(candidate for candidate in DynamicsMode if candidate is not mode)
        try:
            InteractiveSession(RuntimeProblem({vehicle.name: vehicle}), controls=(ControlSpec("command", modes=(wrong_mode.value,)),))
        except ValueError:
            rejected = True
        assert rejected, f"control accepted for invalid mode {wrong_mode.value}"
        print(f"{mode.value}: valid control accepted; invalid mode rejected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
