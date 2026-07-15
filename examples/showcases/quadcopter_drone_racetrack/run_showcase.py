"""Generate the quadcopter/drone racetrack showcase artifacts."""

from __future__ import annotations

from pathlib import Path

from examples.showcases._family_runtime import generate_showcase


def generate(output_dir: str | Path = Path("artifacts/showcases/quadcopter_drone_racetrack")):
    return generate_showcase("quadcopter_drone_racetrack", "quadcopter", output_dir)


if __name__ == "__main__":
    generate()
####
