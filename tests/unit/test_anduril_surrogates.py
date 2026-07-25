from __future__ import annotations

from taoryx.anduril_surrogates import SurrogateControl, SurrogateState, load_surrogate_pack, step_surrogate, validate_surrogate


def test_anduril_pack_contains_the_first_wave() -> None:
    pack = load_surrogate_pack()
    assert set(pack["vehicles"]) == {
        "bolt",
        "bolt_m",
        "anvil",
        "ghost_x",
        "altius_600",
        "altius_700_isr",
        "altius_700m",
        "roadrunner",
        "barracuda_100",
        "barracuda_250",
        "barracuda_500",
        "fury_fq44",
        "omen",
        "thunder",
    }


def test_anduril_pack_declares_energy_state_and_parameter_provenance() -> None:
    """Every promoted configuration exposes the v0.2 data-contract seams."""

    pack = load_surrogate_pack()
    assert pack["parameter_contract"]["evidence_grades"] == ["P", "D", "E", "S"]
    assert set(pack["energy_backends"]) >= {"battery_electric", "fuel_burning", "series_hybrid", "selectable"}
    required = {"energy_backend", "flight_modes", "resource_policy", "parameter_ledger"}
    valid_grades = set(pack["evidence_grades"])
    for vehicle_id, definition in pack["vehicles"].items():
        assert required <= set(definition), vehicle_id
        assert definition["flight_modes"], vehicle_id
        assert definition["parameter_ledger"], vehicle_id
        for record in definition["parameter_ledger"]:
            assert {"path", "value", "unit", "evidence_grade", "source_ref", "uncertainty_policy"} <= set(record), (vehicle_id, record)
            assert record["evidence_grade"] in valid_grades, (vehicle_id, record)
    ####


def test_anduril_selectable_and_hybrid_backends_are_explicit() -> None:
    """Omen/ALTIUS preserve propulsion uncertainty and Thunder is series hybrid."""

    pack = load_surrogate_pack()["vehicles"]
    for vehicle_id in ("altius_600", "altius_700_isr", "altius_700m", "omen"):
        assert pack[vehicle_id]["energy_backend"] == "selectable"
        assert set(pack[vehicle_id]["backend_options"]) == {"battery_electric", "fuel_burning"}
    assert pack["thunder"]["energy_backend"] == "series_hybrid"
    assert pack["thunder"]["claim_boundary"] == "concept_level_only"
    ####


def test_bolt_point_mass_step_is_deterministic() -> None:
    state = SurrogateState()
    control = SurrogateControl(acceleration_m_s2=(2.0, 0.0, 0.0))
    first = step_surrogate("bolt", state, control, 0.5)
    second = step_surrogate("bolt", state, control, 0.5)
    assert first == second
    assert first.position_m == (0.25, 0.0, 0.0)
    assert first.velocity_m_s == (1.0, 0.0, 0.0)


def test_pseudo_profile_adds_attitude_response_state() -> None:
    state = SurrogateState()
    result = step_surrogate(
        "bolt",
        state,
        SurrogateControl(roll_command_rad=0.4, pitch_command_rad=0.1, yaw_rate_rad_s=0.2),
        0.1,
        fidelity="pseudo_6dof",
    )
    assert result.roll_rate_rad_s > 0.0
    assert result.pitch_rate_rad_s > 0.0
    assert result.yaw_rate_rad_s > 0.0


def test_validation_preserves_thunder_concept_boundary() -> None:
    report = validate_surrogate("thunder")
    assert report.passed
    assert any(check.name == "concept_boundary_preserved" for check in report.checks)


def test_all_first_wave_configurations_pass_adapter_smoke_validation() -> None:
    pack = load_surrogate_pack()
    for vehicle_id in pack["vehicles"]:
        report = validate_surrogate(vehicle_id)
        assert report.passed, (vehicle_id, report.failures)


def test_altius_stall_check_reports_the_known_intake_mismatch() -> None:
    report = validate_surrogate("altius_600")
    assert any(check.name == "stall_speed_consistency" for check in report.checks)
