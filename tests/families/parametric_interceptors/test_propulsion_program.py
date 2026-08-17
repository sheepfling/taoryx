"""Focused single- and dual-pulse execution tests for the interceptor plug-in."""

from __future__ import annotations

import pytest
from taoryx_parametric_interceptors import (
    ParametricInterceptorMissionCompositionProvider,
    PropulsionPhase,
    PropulsionProgram,
    ValueOrigin,
    interceptor,
    interceptor_from_catalogue_record,
    pac3_mse_profile,
    resolve_interceptor,
)

from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest


def test_single_pulse_program_keeps_flat_authoring_and_explicit_feedback() -> None:
    resolved = resolve_interceptor(
        interceptor(
            "single-pulse-witness",
            launch_mass_kg=100.0,
            burnout_mass_kg=60.0,
            propulsion_architecture="single_stage_solid",
            thrust_profile_class="neutral",
            burn_time_s=4.0,
        )
    )
    program = PropulsionProgram.from_profile(resolved)

    assert resolved.parameters["burn_time_s"].origin is ValueOrigin.SIMULATION_ASSUMPTION
    assert resolved.number("first_pulse_burn_time_s") == 4.0
    assert resolved.number("second_pulse_burn_time_s") == 0.0
    assert program.sample(0.0).phase is PropulsionPhase.SINGLE_PULSE
    assert program.sample(2.0).mass_kg == pytest.approx(80.0)
    assert program.sample(2.0).throttle_achieved == 1.0
    burnout = program.sample(4.0)
    assert burnout.phase is PropulsionPhase.BURNOUT
    assert burnout.mass_kg == 60.0
    assert burnout.propellant_remaining_kg == 0.0
    assert burnout.available is False
    ####


def test_dual_pulse_program_coasts_then_relights_without_consuming_propellant() -> None:
    resolved = resolve_interceptor(
        interceptor(
            "dual-pulse-witness",
            surrogate_archetype_id="high_energy_dual_pulse_interceptor_v1",
            launch_mass_kg=100.0,
            burnout_mass_kg=60.0,
            thrust_profile_class="neutral",
            first_pulse_burn_time_s=2.0,
            inter_pulse_coast_time_s=3.0,
            second_pulse_burn_time_s=4.0,
            second_pulse_thrust_ratio=0.5,
            second_pulse_propellant_fraction=0.6,
        )
    )
    program = PropulsionProgram.from_profile(resolved)

    assert resolved.number("burn_time_s") == 6.0
    assert resolved.parameters["burn_time_s"].origin is ValueOrigin.SIMULATION_ASSUMPTION
    first_pulse_end = program.sample(2.0)
    late_coast = program.sample(4.9)
    second_pulse = program.sample(5.0)
    burnout = program.sample(9.0)
    assert first_pulse_end.phase is PropulsionPhase.INTER_PULSE_COAST
    assert first_pulse_end.mass_kg == pytest.approx(84.0)
    assert late_coast.mass_kg == first_pulse_end.mass_kg
    assert late_coast.thrust_n == 0.0
    assert late_coast.pulse_index == 0
    assert second_pulse.phase is PropulsionPhase.SECOND_PULSE
    assert second_pulse.available is True
    assert second_pulse.pulse_index == 2
    assert second_pulse.thrust_n > 0.0
    assert burnout.phase is PropulsionPhase.BURNOUT
    assert burnout.mass_kg == 60.0
    assert program.nominal_total_impulse_ns == pytest.approx(resolved.number("thrust_n") * 6.0)
    ####


def test_sparse_pac3_timing_stays_an_archetype_assumption_and_diagnostic() -> None:
    resolved = resolve_interceptor(pac3_mse_profile())

    for name in (
        "first_pulse_burn_time_s",
        "inter_pulse_coast_time_s",
        "second_pulse_burn_time_s",
        "second_pulse_thrust_ratio",
        "second_pulse_propellant_fraction",
    ):
        assert resolved.parameters[name].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert "motor_pulse_timing_missing" in resolved.required_diagnostics
    assert PropulsionProgram.from_profile(resolved).sample(resolved.number("first_pulse_burn_time_s")).phase is PropulsionPhase.INTER_PULSE_COAST
    ####


def test_propulsion_program_rejects_inconsistent_architecture_and_timing() -> None:
    with pytest.raises(ValueError, match="single-pulse propulsion cannot define second-pulse fields"):
        resolve_interceptor(
            interceptor(
                "invalid-single-pulse",
                propulsion_architecture="single_stage_solid",
                second_pulse_burn_time_s=2.0,
            )
        )
    with pytest.raises(ValueError, match="must equal the sum"):
        resolve_interceptor(
            interceptor(
                "invalid-dual-pulse",
                surrogate_archetype_id="high_energy_dual_pulse_interceptor_v1",
                burn_time_s=8.0,
                first_pulse_burn_time_s=3.0,
                second_pulse_burn_time_s=4.0,
            )
        )
    ####


def test_catalogue_motor_timing_alias_flows_into_the_same_flat_profile() -> None:
    profile = interceptor_from_catalogue_record(
        {
            "kind": "interceptor_evidence_record",
            "interceptor_id": "int:test-single-pulse",
            "evidence": {
                "propulsion_architecture": {"value": "single_stage_solid", "origin": "observed"},
                "motor_burn_time": {"value": 5.5, "unit": "s", "origin": "observed"},
            },
        }
    )
    resolved = resolve_interceptor(profile)

    assert resolved.number("burn_time_s") == 5.5
    assert resolved.parameters["burn_time_s"].origin is ValueOrigin.OBSERVED
    assert resolved.number("first_pulse_burn_time_s") == 5.5
    ####


def test_both_fidelity_tiers_emit_the_same_propulsion_phase_contract() -> None:
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor(
            "dual-pulse-composition",
            surrogate_archetype_id="high_energy_dual_pulse_interceptor_v1",
            first_pulse_burn_time_s=0.2,
            inter_pulse_coast_time_s=0.2,
            second_pulse_burn_time_s=0.2,
            second_pulse_thrust_ratio=0.7,
            second_pulse_propellant_fraction=0.5,
            thrust_profile_class="neutral",
        )
    )
    expected_channels = {
        "propulsion.throttle.commanded",
        "propulsion.throttle.achieved",
        "propulsion.propellant.remaining",
        "propulsion.available",
        "propulsion.phase",
        "propulsion.pulse.index",
    }
    assert expected_channels <= {item.id for item in provider.get_model_output_schema("dual-pulse-composition").telemetry_channels}

    phases_by_fidelity: dict[str, list[object]] = {}
    for fidelity in ("point_mass_3dof", "attitude_response_pseudo_6dof"):
        configuration = provider.configuration(
            "dual-pulse-composition",
            fidelity=fidelity,
            runtime_duration_s=0.6,
            runtime_time_step_s=0.05,
        )
        prepared = provider.validate_configuration(configuration)
        response = provider.build_runner().run(
            MissionCompositionRunRequest(
                request_id=f"dual-pulse-{fidelity}",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                output=MissionCompositionOutputSelection(mode="all"),
            )
        )
        samples = response.result.objects[0].samples
        phases_by_fidelity[fidelity] = [sample.values["propulsion.phase"] for sample in samples]
        assert all(expected_channels <= set(sample.values) for sample in samples)
        samples_by_time = {round(sample.time_s, 2): sample for sample in samples}
        assert samples_by_time[0.2].values["propulsion.available"] is False
        assert samples_by_time[0.2].values["propulsion.pulse.index"] == 0
        assert samples_by_time[0.4].values["propulsion.available"] is True
        assert samples_by_time[0.4].values["propulsion.pulse.index"] == 2

    assert phases_by_fidelity["point_mass_3dof"] == phases_by_fidelity["attitude_response_pseudo_6dof"]
    assert phases_by_fidelity["point_mass_3dof"].count("inter_pulse_coast") == 4
    assert phases_by_fidelity["point_mass_3dof"].count("second_pulse") == 4
    ####
