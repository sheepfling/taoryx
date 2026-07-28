"""Run the committed Hummingbird/X8 sensor scenario examples."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files

TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
SCENARIOS = {
    "hummingbird-hover": (
        ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb",
        ("hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl", "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl"),
        ROOT / "examples/sensors/hummingbird_sensorized_hover_v1.yaml",
        12_000,
    ),
    "hummingbird-hover-rotating-earth": (
        ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb",
        ("hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl", "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl"),
        ROOT / "examples/sensors/hummingbird_sensorized_hover_rotating_earth_v1.yaml",
        12_000,
    ),
    "hummingbird-hover-mekf-feedback": (
        ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb",
        ("hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl", "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl"),
        ROOT / "examples/sensors/hummingbird_sensorized_hover_mekf_feedback_v1.yaml",
        12_000,
    ),
    "hummingbird-rate": (
        ROOT / "examples/mission_families/slower_hummingbird/SV05_rate_damped_hover_6dof.prb",
        ("hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl", "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl"),
        ROOT / "examples/sensors/hummingbird_sensorized_hover_v1.yaml",
        12_000,
    ),
    "hummingbird-rotation": (
        ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb",
        ("hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl", "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl"),
        ROOT / "examples/sensors/hummingbird_sensorized_rotation_v1.yaml",
        12_000,
    ),
    "hummingbird-translation": (
        ROOT / "examples/mission_families/slower_hummingbird/SV05_3dof.prb",
        ("hummingbird_rotor_static.tbl",),
        ROOT / "examples/sensors/hummingbird_sensorized_translation_v1.yaml",
        2_500,
    ),
    "hummingbird-pseudo6dof": (
        ROOT / "examples/mission_families/slower_hummingbird/SV05_3dof.prb",
        ("hummingbird_rotor_static.tbl",),
        ROOT / "examples/sensors/hummingbird_sensorized_pseudo6dof_v1.yaml",
        2_500,
    ),
    "x8-powered": (
        ROOT / "examples/mission_families/slower_x8/SV03_source_trim_hold_30_6dof.prb",
        ("skywalker_x8_static_6axis.tbl", "skywalker_x8_collective_elevon_6axis.tbl", "skywalker_x8_differential_elevon_6axis.tbl", "skywalker_x8_thrust.tbl"),
        ROOT / "examples/sensors/x8_sensorized_powered_v1.yaml",
        13_000,
    ),
    "x8-powered-hg9900": (
        ROOT / "examples/mission_families/slower_x8/SV03_source_trim_hold_30_6dof.prb",
        ("skywalker_x8_static_6axis.tbl", "skywalker_x8_collective_elevon_6axis.tbl", "skywalker_x8_differential_elevon_6axis.tbl", "skywalker_x8_thrust.tbl"),
        ROOT / "examples/sensors/x8_sensorized_powered_mekf_feedback_v1.yaml",
        13_000,
    ),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=tuple(SCENARIOS))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-steps", type=int)
    arguments = parser.parse_args()
    problem, table_names, sensor_spec, default_max_steps = SCENARIOS[arguments.scenario]
    report = run_files(
        problem,
        tuple(TABLE_ROOT / name for name in table_names),
        output_dir=arguments.output_dir,
        max_steps=default_max_steps if arguments.max_steps is None else arguments.max_steps,
        profile=GrammarProfile.TAORYX,
        sensor_spec=sensor_spec,
    )
    print(json.dumps(report.as_dict(), indent=2, sort_keys=True))
    return report.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
