from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[2]
HUMMINGBIRD_TABLES = tuple(
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables" / name
    for name in (
        "hummingbird_cx.tbl",
        "hummingbird_cy.tbl",
        "hummingbird_cz.tbl",
        "hummingbird_cmx.tbl",
        "hummingbird_cmy.tbl",
        "hummingbird_cmz.tbl",
    )
)
X8_TABLES = tuple(
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables" / name
    for name in (
        "skywalker_x8_static_6axis.tbl",
        "skywalker_x8_collective_elevon_6axis.tbl",
        "skywalker_x8_differential_elevon_6axis.tbl",
        "skywalker_x8_thrust.tbl",
    )
)
HUMMINGBIRD_TRANSLATION_TABLES = (
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/hummingbird_rotor_static.tbl",
)

pytestmark = [pytest.mark.slow, pytest.mark.integration]


def test_hummingbird_sensorized_hover_artifact_is_complete(tmp_path: Path) -> None:
    report = run_files(
        ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb",
        HUMMINGBIRD_TABLES,
        output_dir=tmp_path,
        max_steps=12_000,
        profile=GrammarProfile.TAORYX,
        sensor_spec=ROOT / "examples/sensors/hummingbird_sensorized_hover_v1.yaml",
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    execution = report.artifacts[0].sensor_execution
    assert execution["scenario_identity"] == "hummingbird_sensorized_hover_v1"
    assert execution["truth_contract"]["earth_rate"]["mode"] == "source"
    assert execution["measurement_summary"]["timeout"] is False
    assert execution["measurement_summary"]["dropped"] == 0
    assert len(execution["plots"]["plots"]) >= 6
    assert any(path.endswith("plot-manifest.json") for path in report.outputs)


def test_hummingbird_rotation_only_artifact_is_translation_independent(tmp_path: Path) -> None:
    report = run_files(
        ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb",
        HUMMINGBIRD_TABLES,
        output_dir=tmp_path,
        max_steps=12_000,
        profile=GrammarProfile.TAORYX,
        sensor_spec=ROOT / "examples/sensors/hummingbird_sensorized_rotation_v1.yaml",
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    execution = report.artifacts[0].sensor_execution
    assert execution["spec"]["truth_mode"] == "rotation-only"
    assert "attitude_dead_reckoning" in execution["estimators"]
    assert execution["measurements"]["gyro"][1]["payload_contract"]["measurement"] == "angular-rate"
    assert execution["measurement_summary"]["timeout"] is False
    assert any(plot["name"] == "attitude-estimates" for plot in execution["plots"]["plots"])


def test_x8_sensorized_powered_short_gate(tmp_path: Path) -> None:
    report = run_files(
        ROOT / "examples/mission_families/slower_x8/SV03_source_trim_hold_30_6dof.prb",
        X8_TABLES,
        output_dir=tmp_path,
        max_steps=13_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
        sensor_spec=ROOT / "examples/sensors/x8_sensorized_powered_v1.yaml",
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    summary = report.artifacts[0].sensor_execution["measurement_summary"]
    assert summary["valid"] > 3_000
    assert summary["timeout"] is False


@pytest.mark.parametrize(
    ("scenario", "truth_mode", "estimator"),
    (
        ("hummingbird_sensorized_translation_v1.yaml", "translation-only", "translation_dead_reckoning"),
        ("hummingbird_sensorized_pseudo6dof_v1.yaml", "pseudo-6dof", "mekf"),
    ),
)
def test_hummingbird_lower_fidelity_sensor_contracts(tmp_path: Path, scenario: str, truth_mode: str, estimator: str) -> None:
    report = run_files(
        ROOT / "examples/mission_families/slower_hummingbird/SV05_3dof.prb",
        HUMMINGBIRD_TRANSLATION_TABLES,
        output_dir=tmp_path,
        max_steps=2_500,
        profile=GrammarProfile.TAORYX,
        sensor_spec=ROOT / "examples/sensors" / scenario,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    execution = report.artifacts[0].sensor_execution
    assert execution["truth_contract"]["mode"] == truth_mode
    assert estimator in execution["estimators"]
    assert execution["measurement_summary"]["timeout"] is False
