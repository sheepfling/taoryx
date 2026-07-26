from pathlib import Path

import numpy as np
import pytest

from taoryx.contracts import Vector3
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.problem_parser import parse_problem_text
from taoryx.runtime import (
    GainScheduledLqrController,
    LqrController,
    LqrUncertaintySpec,
    assess_lqr_robustness,
    solve_continuous_lqr,
    solve_scaled_continuous_lqr,
)
from taoryx.runtime.lowering import RuntimeTable, _build_attitude_lqr
from taoryx.runtime.program import LoadedProgram
from taoryx.tables import ExtrapolationMode, prepare_table


def test_continuous_lqr_solves_and_stabilizes_double_integrator() -> None:
    result = solve_continuous_lqr(
        ((0.0, 1.0), (0.0, 0.0)),
        ((0.0,), (1.0,)),
        ((1.0, 0.0), (0.0, 1.0)),
        ((1.0,),),
        state_names=("position", "velocity"),
        control_names=("acceleration",),
    )

    assert result.gain.shape == (1, 2)
    assert result.controllable is True
    assert np.all(np.real(result.closed_loop_eigenvalues) < 0.0)
    assert result.hurwitz is True
    assert result.maximum_real_pole < 0.0
    assert result.unstable_poles == ()
    assert result.state_names == ("position", "velocity")


def test_scaled_lqr_returns_a_physical_gain_with_dimensionless_poles() -> None:
    result = solve_scaled_continuous_lqr(
        ((0.0, 1.0), (0.0, 0.0)),
        ((0.0,), (1.0,)),
        ((1.0, 0.0), (0.0, 1.0)),
        ((1.0,),),
        state_scales=(2.0, 4.0),
        control_scales=(3.0,),
        state_names=("position", "velocity"),
        control_names=("acceleration",),
    )

    assert result.hurwitz
    assert result.gain.shape == (1, 2)
    assert result.gain[0, 0] > 0.0
    assert result.gain[0, 1] > 0.0
    ####


def test_lqr_controller_closes_double_integrator_with_named_bounded_command() -> None:
    result = solve_continuous_lqr(
        ((0.0, 1.0), (0.0, 0.0)),
        ((0.0,), (1.0,)),
        ((1.0, 0.0), (0.0, 1.0)),
        ((1.0,),),
        state_names=("position", "velocity"),
        control_names=("acceleration",),
    )
    controller = LqrController(result, upper={"acceleration": 0.5}, lower={"acceleration": -0.5})

    command = controller.command({"position": 2.0, "velocity": 0.0})

    assert command.controls["acceleration"] == pytest.approx(-0.5)
    assert command.unsaturated["acceleration"] < -0.5
    assert command.saturated == ("acceleration",)


def test_lqr_controller_rejects_missing_named_state() -> None:
    result = solve_continuous_lqr(
        ((0.0, 1.0), (0.0, 0.0)),
        ((0.0,), (1.0,)),
        ((1.0, 0.0), (0.0, 1.0)),
        ((1.0,),),
        state_names=("position", "velocity"),
        control_names=("acceleration",),
    )
    with pytest.raises(KeyError, match="velocity"):
        LqrController(result).command({"position": 1.0})


def test_rigid_body_lowering_uses_declared_attitude_lqr(tmp_path: Path) -> None:
    source = tmp_path / "attitude_lqr.prb"
    source.write_text(
        "(attitude-lqr)\n"
        "*mode rigid-body-6dof\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*runtime status vehicle reference-area=1 reference-length=1 inertia-x=2 inertia-y=3 inertia-z=4 dry-mass-kg=10\n"
        "*runtime status target latitude-deg=0 longitude-deg=0 altitude-m=0 velocity-east-mps=0 velocity-north-mps=0\n"
        "*runtime status route mode=great-circle start-latitude-deg=0 start-longitude-deg=0 duration-s=1 powered-end-s=1 powered-speed-mps=10 powered-climb-rate-mps=0\n"
        "*runtime status guidance propnav-gain=1.0 propnav-attitude-horizon-s=1.0\n"
        "*runtime status actuator maximum-moment=100 maximum-body-rate-deg-s=3600\n"
        "*runtime lqr attitude states=attitude-error-x,attitude-error-y,attitude-error-z,wx,wy,wz controls=moment-x,moment-y,moment-z q-angle=2 q-rate=1 r-moment=1 method=continuous\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecic x=6378137 y=0 z=0 xdt=0 ydt=10 zdt=0 qw=1 qx=0 qy=0 qz=0 wx=0 wy=0 wz=0 time=0 mass=10 propellant_mass=0\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.1\n"
        "    *fly propnav=1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)
    problem = program.case()
    vehicle = problem.vehicles["1"]

    assert problem.metadata["lqr"]["q-angle"] == "2"
    assert vehicle.derivative is not None
    rates = vehicle.derivative(vehicle.state)
    assert len(rates) == len(vehicle.state.values)


def test_attitude_lqr_uses_flattened_matrix_tables() -> None:
    document = parse_problem_text(
        "(lqr)\n"
        "*runtime lqr attitude states=attitude-error-x,attitude-error-y,attitude-error-z,wx,wy,wz controls=moment-x,moment-y,moment-z a-table=A b-table=B q-table=Q r-table=R\n"
        "*end\n",
        profile=GrammarProfile.TAORYX,
    )

    def table(name: str, values: tuple[float, ...]) -> RuntimeTable:
        return RuntimeTable(name, "output", ("index",), "output", prepare_table((tuple(range(len(values))),), values, extrapolation=ExtrapolationMode.CLAMP))

    matrices = {
        "a": table("A", (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) + (0.0,) * 18),
        "b": table("B", (0.0,) * 9 + (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)),
        "q": table("Q", (1.0,) * 6),
        "r": table("R", (1.0,) * 3),
    }
    matrices["b"] = table("B", (0.0,) * 9 + (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))

    controller = _build_attitude_lqr(document.problems[0], Vector3(2.0, 3.0, 4.0), {}, matrices)
    assert controller is not None
    assert controller.result.gain.shape == (3, 6)
    assert controller.nominal.realization is not None
    assert controller.nominal.realization.a_sha256 is not None
    assert controller.nominal.realization.control_path[-1] == "plant"


def test_rate_lqr_uses_body_rates_and_rate_realization_contract() -> None:
    """The rate-loop witness is three-state and carries physical provenance."""

    document = parse_problem_text(
        "(rate-lqr)\n"
        "*runtime lqr rate states=wx,wy,wz controls=moment-x,moment-y,moment-z q-rate=1 r-moment=0.5\n"
        "*end\n",
        profile=GrammarProfile.TAORYX,
    )

    controller = _build_attitude_lqr(
        document.problems[0],
        Vector3(0.00365, 0.00368, 0.00703),
        {"maximum-moment": "1", "allocator": "hummingbird-quad-x"},
        {},
        initial_mass_kg=0.5,
        controller_name="rate",
    )
    assert controller is not None
    assert controller.result.gain.shape == (3, 3)
    assert controller.result.state_names == ("wx", "wy", "wz")
    assert controller.nominal.realization is not None
    assert controller.nominal.realization.role == "rate"
    assert tuple(channel.unit for channel in controller.nominal.realization.states) == ("rad/s",) * 3
    assert controller.nominal.realization.control_path == ("guidance", "rate_lqr", "allocator", "actuator", "plant")
    assert controller.result.hurwitz


def test_profile_attitude_lqr_uses_nominal_mass_ratio_and_inertia_scale() -> None:
    """A catalog profile schedules the normalized design from both mass and inertia."""

    document = parse_problem_text(
        "(profile-lqr)\n"
        "*runtime lqr attitude profile=skywalker-x8-standard "
        "states=attitude-error-x,attitude-error-y,attitude-error-z,wx,wy,wz "
        "controls=moment-x,moment-y,moment-z\n"
        "*end\n",
        profile=GrammarProfile.TAORYX,
    )
    controller = _build_attitude_lqr(document.problems[0], Vector3(0.325, 0.140, 0.400), {}, {})
    assert controller is not None
    state = {name: 0.01 for name in ("attitude-error-x", "attitude-error-y", "attitude-error-z", "wx", "wy", "wz")}
    controller.command(state, mass_kg=3.364, inertia=(0.325, 0.140, 0.400))
    nominal_gain = np.asarray(controller.result.gain)
    controller.command(state, mass_kg=6.728, inertia=(0.325, 0.140, 0.400))
    assert controller.schedule_updates == 2
    assert not np.allclose(nominal_gain, controller.result.gain)


@pytest.mark.parametrize(
    "matrices, message",
    [
        ((((0.0,),), ((1.0,), (1.0,)), ((1.0,),), ((1.0,),)), "same row count"),
        ((((0.0, 1.0),), ((1.0,),), ((1.0,),), ((1.0,),)), "square"),
        ((((0.0,),), ((1.0,),), ((1.0,),), ((0.0,),)), "positive definite"),
    ],
)
def test_lqr_rejects_invalid_matrix_contract(matrices: tuple[object, ...], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        solve_continuous_lqr(*matrices)  # type: ignore[arg-type]


def test_gain_scheduled_lqr_rebuilds_when_inertia_operating_point_changes() -> None:
    """Mass-property changes select a new plant-scaled gain, not a guessed exponent."""

    a = ((0.0, 1.0), (0.0, 0.0))
    q = ((1.0, 0.0), (0.0, 1.0))
    r = ((1.0,),)

    def build(_mass: float | None, inertia: tuple[float, float, float] | None) -> LqrController:
        value = 1.0 if inertia is None else inertia[0]
        result = solve_continuous_lqr(
            a,
            ((0.0,), (1.0 / value,)),
            q,
            r,
            state_names=("angle", "rate"),
            control_names=("moment",),
        )
        return LqrController(result)

    nominal = build(10.0, (1.0, 1.0, 1.0))
    scheduled = GainScheduledLqrController(nominal, builder=build)
    scheduled.command({"angle": 0.1, "rate": 0.0}, mass_kg=10.0, inertia=(1.0, 1.0, 1.0))
    first_gain = np.asarray(scheduled.result.gain)
    assert scheduled.schedule_updates == 1
    scheduled.command({"angle": 0.1, "rate": 0.0}, mass_kg=9.0, inertia=(2.0, 1.0, 1.0))
    assert scheduled.schedule_updates == 2
    assert not np.allclose(first_gain, scheduled.result.gain)


def test_lqr_uncertainty_screen_reports_sampled_pole_margin() -> None:
    """Estimated derivatives can be screened without pretending to be exact."""

    a = ((0.0, 1.0), (0.0, 0.0))
    b = ((0.0,), (1.0,))
    result = solve_continuous_lqr(a, b, ((1.0, 0.0), (0.0, 1.0)), ((1.0,),))
    report = assess_lqr_robustness(a, b, result, LqrUncertaintySpec(a_fraction=0.1, b_fraction=0.1, samples=17))
    assert report.samples == 17
    assert report.nominal_max_real_pole < 0.0
    assert report.worst_max_real_pole >= report.nominal_max_real_pole
