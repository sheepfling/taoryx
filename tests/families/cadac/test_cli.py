from __future__ import annotations

import json
from pathlib import Path

import pytest
from taoryx.families.cadac.__main__ import main
from test_ads6_aircraft import _write_ads6_aircraft_case
from test_ads6_engagement import _write_aircraft_engagement
from test_ads6_sam import _write_ads6_sam_case
from test_ads6_srbm import _write_ads6_srbm_case
from test_agm6 import _write_agm6_case
from test_cruise5 import _case as _write_cruise_case
from test_ghame3 import _case as _write_ghame3_case
from test_ghame6 import _write_ghame6_case
from test_rocket6g import _write_rocket_case
from test_rocket6g_rcs import _case as _write_rocket_rcs_case
from test_sraam6 import _write_sraam6_case


def test_catalog_cli_emits_one_selected_package(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(("catalog", "--package", "AIM5")) == 0

    output = capsys.readouterr().out
    payload = json.loads(output)
    assert payload["package_id"] == "AIM5"
    assert payload["actors"][0]["phases"][0]["taoryx_tier"] == "pseudo_6dof"


####


def test_plugin_cli_emits_actor_descriptors(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(("plugins",)) == 0

    payload = json.loads(capsys.readouterr().out)
    assert len(payload["plugins"]) == 19
    assert any(item["plugin_id"] == "cadac.aim5.missile" for item in payload["plugins"])


####


def test_provider_catalog_cli_emits_dynamic_taoryx_models(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(("provider-catalog",)) == 0

    payload = json.loads(capsys.readouterr().out)
    assert payload["provider"]["id"] == "cadac"
    assert payload["provider"]["model_count"] == 17
    assert any(item["id"] == "cadac.falcon6.aircraft" for item in payload["models"])


####


def test_cruise5_lower_cli_reports_pseudo6dof_source_model(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(("cruise5-lower", str(_write_cruise_case(tmp_path)))) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["taoryx_tier"] == "pseudo_6dof"
    assert payload["source_model"] == "CRUISE3"


####


def test_ghame3_lower_cli_reports_point_mass_source_model(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(("ghame3-lower", str(_write_ghame3_case(tmp_path)))) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["taoryx_tier"] == "point_mass_3dof"
    assert payload["source_model"] == "CRUISE3"


####


def test_rocket6g_rcs_lower_cli_keeps_direct_wrench_claim(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(("rocket6g-rcs-lower", str(_write_rocket_rcs_case(tmp_path)))) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["taoryx_tier"] == "rigid_body_6dof_direct_wrench"
    assert payload["control_realization"] == "direct_wrench"


####


def test_magsix_cli_lower_and_run(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    from test_magsix import _case as write_magsix_case

    path = write_magsix_case(tmp_path)
    assert main(("magsix-lower", str(path))) == 0
    lower_output = capsys.readouterr().out
    assert '"source_model": "ROTOR"' in lower_output
    assert main(("magsix-run", str(path), "--end-time-dnt", "0.01", "--sample-step-dnt", "0.01")) == 0
    run_output = capsys.readouterr().out
    assert '"status": "completed"' in run_output


####


def test_rocket6g_cli_lower_and_run_phase_aware_vehicle(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = _write_rocket_case(tmp_path)
    assert main(("rocket6g-lower", str(source))) == 0
    lower_payload = json.loads(capsys.readouterr().out)
    assert lower_payload["control_realization"] == "mixed_effector"
    assert [stage["stage_number"] for stage in lower_payload["stages"]] == [1, 2, 3]

    assert (
        main(
            (
                "rocket6g-run",
                str(source),
                "--end-time",
                "1.0",
                "--sample-step",
                "0.1",
                "--tvc-pitch-deg",
                "1.0",
            )
        )
        == 0
    )
    run_payload = json.loads(capsys.readouterr().out)
    assert run_payload["terminated_reason"] == "end_time"
    assert any(sample["source_phase"] == "mixed_tvc_rcs" for sample in run_payload["samples"])


####


def test_sraam6_cli_lower_and_run_physical_fin_engagement(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = _write_sraam6_case(tmp_path)
    assert main(("sraam6-lower", str(source))) == 0
    lower_payload = json.loads(capsys.readouterr().out)
    assert lower_payload["taoryx_tier"] == "rigid_body_6dof_surface_allocated"
    assert lower_payload["source_model"] == "MISSILE6"
    assert lower_payload["vehicle_order"] == ["MISSILE6", "TARGET3"]

    assert main(("sraam6-run", str(source), "--end-time", "0.03", "--sample-step", "0.01")) == 0
    run_payload = json.loads(capsys.readouterr().out)
    assert run_payload["terminated_reason"] == "end_time"
    assert len(run_payload["samples"][0]["requested_fins_deg"]) == 4
    assert len(run_payload["target_samples"]) >= 2


####


def test_agm6_cli_lower_and_run_three_actor_physical_fin_engagement(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = _write_agm6_case(tmp_path)
    assert main(("agm6-lower", str(source))) == 0
    lower_payload = json.loads(capsys.readouterr().out)
    assert lower_payload["taoryx_tier"] == "rigid_body_6dof_surface_allocated"
    assert lower_payload["source_model"] == "MISSILE6"
    assert lower_payload["vehicle_order"] == ["MISSILE6", "TARGET3", "AIRCRAFT3"]

    assert main(("agm6-run", str(source), "--end-time", "0.04", "--sample-step", "0.01", "--random-seed", "7")) == 0
    run_payload = json.loads(capsys.readouterr().out)
    assert run_payload["terminated_reason"] == "end_time"
    assert len(run_payload["samples"][0]["requested_fins_deg"]) == 4
    assert len(run_payload["target_samples"]) >= 2
    assert len(run_payload["aircraft_samples"]) >= 2


####


def test_ghame6_cli_lower_and_run_phase_aware_three_actor_mission(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = _write_ghame6_case(tmp_path)
    assert main(("ghame6-lower", str(source))) == 0
    lower_payload = json.loads(capsys.readouterr().out)
    assert lower_payload["actor_order"] == ["HYPER6", "SAT3", "RADAR0"]
    assert lower_payload["taoryx_tier"] == "rigid_body_6dof_surface_allocated"
    assert "tvc" not in lower_payload["module_order"]

    assert (
        main(
            (
                "ghame6-run",
                str(source),
                "--end-time",
                "1.2",
                "--sample-step",
                "0.05",
                "--aileron-deg",
                "1.0",
                "--elevator-deg",
                "2.0",
                "--boost-cutoff-time",
                "0.65",
                "--terminal-lock-time",
                "0.85",
                "--random-seed",
                "7",
            )
        )
        == 0
    )
    run_payload = json.loads(capsys.readouterr().out)
    assert run_payload["terminated_reason"] == "end_time"
    assert {sample["runtime_fidelity"] for sample in run_payload["samples"]} == {
        "rigid_body_6dof_surface_allocated",
        "rigid_body_6dof_direct_wrench",
    }
    assert len(run_payload["satellite_samples"]) == len(run_payload["samples"])
    assert run_payload["radar_update_count"] > 0


####


def test_ads6_sam_cli_lower_and_run_each_realization(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = _write_ads6_sam_case(tmp_path)
    assert main(("ads6-sam-lower", str(source))) == 0
    lower_payload = json.loads(capsys.readouterr().out)
    assert lower_payload["initial_state"]["speed_mps"] == 300.0
    assert lower_payload["fin_actuator"]["mode"] == 2
    assert lower_payload["tvc"]["mode"] == 2
    assert lower_payload["rcs"]["moment_mode"] == 21

    assert (
        main(
            (
                "ads6-sam-run",
                str(source),
                "--phase",
                "tvc_control",
                "--pitch-deg",
                "1.0",
                "--end-time",
                "0.01",
                "--sample-step",
                "0.005",
            )
        )
        == 0
    )
    run_payload = json.loads(capsys.readouterr().out)
    assert run_payload["source_phase"] == "tvc_control"
    assert run_payload["samples"][-1]["fidelity"] == "rigid_body_6dof_surface_allocated"


####


def test_ads6_srbm_cli_lower_and_run_response_law_vehicle(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = _write_ads6_srbm_case(tmp_path)
    assert main(("ads6-srbm-lower", str(source))) == 0
    lower_payload = json.loads(capsys.readouterr().out)
    assert lower_payload["source_model"] == "ROCKET5"
    assert lower_payload["taoryx_tier"] == "pseudo_6dof"
    assert lower_payload["control_realization"] == "response_law"

    assert main(("ads6-srbm-run", str(source), "--end-time", "0.05", "--sample-step", "0.05")) == 0
    run_payload = json.loads(capsys.readouterr().out)
    assert run_payload["terminated_reason"] == "end_time"
    assert run_payload["samples"][-1]["fidelity"] == "pseudo_6dof"
    assert "quaternion_wxyz" not in run_payload["samples"][-1]


####


def test_ads6_aircraft_cli_lower_and_run_point_mass_target(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = _write_ads6_aircraft_case(tmp_path)
    assert main(("ads6-aircraft-lower", str(source))) == 0
    lower_payload = json.loads(capsys.readouterr().out)
    assert lower_payload["source_model"] == "AIRCRAFT3"
    assert lower_payload["taoryx_tier"] == "point_mass_3dof"
    assert lower_payload["control_realization"] == "force_model"

    assert main(("ads6-aircraft-run", str(source), "--end-time", "0.05", "--sample-step", "0.05")) == 0
    run_payload = json.loads(capsys.readouterr().out)
    assert run_payload["terminated_reason"] == "end_time"
    assert run_payload["samples"][-1]["fidelity"] == "point_mass_3dof"
    assert "quaternion_wxyz" not in run_payload["samples"][-1]


####


def test_ads6_engagement_cli_lower_and_run_source_ordered_package(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = _write_aircraft_engagement(tmp_path)
    assert main(("ads6-engagement-lower", str(source))) == 0
    lower_payload = json.loads(capsys.readouterr().out)
    assert lower_payload["target_kind"] == "aircraft"
    assert [item["actor_id"] for item in lower_payload["source_order"]] == ["m1", "a1", "f1"]

    assert (
        main(
            (
                "ads6-engagement-run",
                str(source),
                "--end-time",
                "0.03",
                "--sample-step",
                "0.01",
                "--command-law",
                "hold",
                "--radar-seed",
                "7",
            )
        )
        == 0
    )
    run_payload = json.loads(capsys.readouterr().out)
    assert run_payload["source_order"] == ["m1", "a1", "f1"]
    assert len(run_payload["objects"]) == 3
    assert any(item["kind"] == "launch_command" for item in run_payload["events"])
    assert any(item["kind"] == "missile_launch" for item in run_payload["events"])


####
