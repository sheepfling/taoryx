"""Generate the orbital insertion/coast/reentry showcase artifacts."""

from __future__ import annotations

from pathlib import Path

from examples.showcases._family_runtime import generate_showcase


def generate(output_dir: str | Path = Path("artifacts/showcases/orbital_insertion_coast_reentry")):
    return generate_showcase("orbital_insertion_coast_reentry", "orbital", output_dir)


if __name__ == "__main__":
    generate()
####
