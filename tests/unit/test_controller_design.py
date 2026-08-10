from __future__ import annotations

from pathlib import Path

import numpy as np

from taoryx.controller_design import ControllerDesignCatalog, ControllerDesignSpec, build_lqi_controller, build_lqr_controller, load_controller_catalog
from taoryx.trim import TrimSpec, solve_trim


def test_controller_design_catalog_rejects_cross_channel_reuse() -> None:
    try:
        ControllerDesignSpec(
            id="bad",
            method="lqr",
            trim="trim-v1",
            allocator="vehicle",
            states=("alpha",),
            controls=("alpha",),
        )
    except ValueError as error:
        assert "overlaps" in str(error)
    else:
        raise AssertionError("state/control name reuse should be rejected")


def test_lqr_factory_requires_trim_and_design_channels_to_match() -> None:
    trim_spec = TrimSpec(
        state_names=("x",),
        control_names=("u",),
        residual_names=("equilibrium",),
        state_initial={"x": 0.0},
        control_initial={"u": 0.0},
    )
    trim = solve_trim(trim_spec, lambda state, controls: {"equilibrium": state["x"] + controls["u"]})
    design = ControllerDesignSpec(
        id="local-lqr",
        method="lqr",
        trim="trim-v1",
        allocator="vehicle",
        states=("x",),
        controls=("u",),
    )

    controller = build_lqr_controller(design, trim, [[0.0]], [[1.0]], [[1.0]], [[1.0]])

    command = controller.command({"x": 1.0})
    assert command.controls["u"] < 0.0
    np.testing.assert_allclose(controller.result.gain, [[1.0]], atol=1e-10)


def test_controller_design_catalog_ids_are_unique() -> None:
    try:
        ControllerDesignCatalog(
            schema_version=1,
            id="catalog",
            designs=(
                ControllerDesignSpec(id="same", method="pid", trim="t", allocator="a"),
                ControllerDesignSpec(id="same", method="mpc", trim="t", allocator="a"),
            ),
        )
    except ValueError as error:
        assert "unique" in str(error)
    else:
        raise AssertionError("duplicate design IDs should be rejected")


def test_lqi_factory_records_integral_output_and_tracks_the_declared_reference() -> None:
    trim_spec = TrimSpec(
        state_names=("position", "velocity"),
        control_names=("acceleration",),
        residual_names=("equilibrium",),
        state_initial={"position": 0.0, "velocity": 0.0},
        control_initial={"acceleration": 0.0},
    )
    trim = solve_trim(trim_spec, lambda state, controls: {"equilibrium": state["position"] + controls["acceleration"]})
    design = ControllerDesignSpec(
        id="offset-free-position",
        method="lqi",
        trim="trim-v1",
        allocator="vehicle",
        states=("position", "velocity"),
        controls=("acceleration",),
        integral_outputs=("position",),
        implementation_version="v1",
        plant_source="test",
        linearization_source="test",
        q_id="test-q",
        r_id="test-r",
        state_scale_id="test-state",
        control_scale_id="test-control",
    )

    controller = build_lqi_controller(
        design,
        trim,
        ((0.0, 1.0), (0.0, 0.0)),
        ((0.0,), (1.0,)),
        ((5.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 10.0)),
        ((1.0,),),
        output_matrix=((1.0, 0.0),),
    )

    controller.command({"position": 0.2, "velocity": 0.0}, {"position": 0.0}, dt=0.1)
    assert controller.integral_error["position"] > 0.0
    assert controller.result.design.hurwitz
    assert controller.realization is not None
    assert controller.realization.implementation == "lqi"
    assert controller.realization.integral_states == ("position",)


def test_repository_controller_design_catalog_is_loadable() -> None:
    catalog = load_controller_catalog(Path("verification/controller_designs.yaml"))

    assert catalog.get("b747-local-lqr").method == "lqr"
    assert catalog.get("hummingbird-rate-lqr").allocator == "hummingbird-quad-x"
