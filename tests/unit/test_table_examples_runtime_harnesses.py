from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from taoryx.runtime.runner import run_files

TABLE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "table_examples_v1"
RUNTIME_ROOT = TABLE_ROOT / "runtime"


def test_table_example_runtime_manifest_lists_every_harness() -> None:
    manifest = yaml.safe_load((RUNTIME_ROOT / "manifest.yaml").read_text(encoding="utf-8"))
    listed = sorted(entry["problem_file"] for entry in manifest["harnesses"])
    actual = sorted(path.name for path in RUNTIME_ROOT.glob("*.prb"))

    assert listed == actual


@pytest.mark.parametrize("problem_name", sorted(path.name for path in RUNTIME_ROOT.glob("*.prb")))
def test_table_example_runtime_harnesses_execute(problem_name: str, tmp_path: Path) -> None:
    manifest = yaml.safe_load((RUNTIME_ROOT / "manifest.yaml").read_text(encoding="utf-8"))
    entry = next(item for item in manifest["harnesses"] if item["problem_file"] == problem_name)
    problem = RUNTIME_ROOT / problem_name
    tables = tuple(TABLE_ROOT / relative for relative in entry["tables"])

    report = run_files(problem, tables, output_dir=tmp_path / problem.stem, max_steps=200)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.cases >= 1
    assert report.outputs, problem_name
    assert report.results and all(result.completed for result in report.results), problem_name
    ####


@pytest.mark.parametrize(
    ("problem_name", "expected"),
    [
        (
            "air_breathing_runtime.prb",
            {
                "throttle_limited_thrust_sample": pytest.approx(2400.0),
                "throttle_limited_mdot_sample": pytest.approx(0.064),
                "inlet_recovery_sample": pytest.approx(0.96),
            },
        ),
        (
            "control_surfaces_runtime.prb",
            {
                "flap_sample": pytest.approx(0.41),
                "fin_pitch_sample": pytest.approx(0.5),
                "fin_yaw_sample": pytest.approx(0.5),
                "fin_roll_sample": pytest.approx(0.25),
            },
        ),
    ],
)
def test_table_example_runtime_harness_samples_are_deterministic(
    problem_name: str, expected: dict[str, object], tmp_path: Path
) -> None:
    manifest = yaml.safe_load((RUNTIME_ROOT / "manifest.yaml").read_text(encoding="utf-8"))
    entry = next(item for item in manifest["harnesses"] if item["problem_file"] == problem_name)
    problem = RUNTIME_ROOT / problem_name
    tables = tuple(TABLE_ROOT / relative for relative in entry["tables"])

    report = run_files(problem, tables, output_dir=tmp_path / problem.stem, max_steps=200)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    final = report.results[0].states["1"][-1].named
    if problem_name == "air_breathing_runtime.prb":
        assert final["throttle_limited_thrust_sample"] > 2_300.0
        assert final["throttle_limited_mdot_sample"] > 0.0
        assert final["afterburning_thrust_sample"] > 0.0
        assert final["afterburning_mdot_sample"] > 0.0
        assert final["inlet_recovery_sample"] == pytest.approx(0.96, abs=0.03)
        return
    for name, value in expected.items():
        assert final[name] == value
    ####
