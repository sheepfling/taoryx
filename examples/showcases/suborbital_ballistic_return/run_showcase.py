"""Generate the suborbital ballistic-return showcase artifacts."""

from __future__ import annotations

from pathlib import Path

from examples.showcases._family_runtime import generate_showcase


def generate(output_dir: str | Path = Path("artifacts/showcases/suborbital_ballistic_return")):
    return generate_showcase("suborbital_ballistic_return", "suborbital", output_dir)


if __name__ == "__main__":
    generate()
####
