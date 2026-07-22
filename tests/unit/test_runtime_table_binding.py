from __future__ import annotations

import math
from pathlib import Path

import pytest

from taoryx.language.ingest import ingest_file
from taoryx.language.models import TableDocument
from taoryx.runtime.table_binding import bind_runtime_tables
from taoryx.vehicle import AeroQueryContext, PreparedAerodynamicCoefficients

ROOT = Path(__file__).resolve().parents[2]
TABLES = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"


def test_x8_coefficient_families_get_deterministic_runtime_aliases() -> None:
    paths = (
        TABLES / "skywalker_x8_static_6axis.tbl",
        TABLES / "skywalker_x8_collective_elevon_6axis.tbl",
        TABLES / "skywalker_x8_differential_elevon_6axis.tbl",
    )
    documents = tuple(TableDocument.model_validate(ingest_file(path).document) for path in paths)

    available, lowered = bind_runtime_tables(documents, {})

    assert {"cx-static", "cx-collective", "cx-differential"}.issubset(available)
    assert "cx" in available
    assert lowered["cx"].name == "cx"
    assert lowered["cx-collective"].independent_variables[-1] == "collective_elevon"
    assert lowered["cx-differential"].independent_variables[-1] == "differential_elevon"
    assert lowered["cx-static"].independent_variables == ("velocity_m_s", "altitude_m", "alpha", "beta")


def test_x8_control_families_compose_as_zero_referenced_increments() -> None:
    """The common adapter preserves static coefficients at zero control.

    The source control decks contain absolute six-axis coefficients, while a
    problem file supplies the static deck and one or more control commands.
    The runtime contract is therefore ``static + (control - control@zero)``.
    This test exercises that contract with the real imported X8 source tables
    rather than duplicating any coefficient values in the test.
    """

    paths = (
        TABLES / "skywalker_x8_static_6axis.tbl",
        TABLES / "skywalker_x8_collective_elevon_6axis.tbl",
        TABLES / "skywalker_x8_differential_elevon_6axis.tbl",
    )
    documents = tuple(TableDocument.model_validate(ingest_file(path).document) for path in paths)
    _, lowered = bind_runtime_tables(documents, {})
    composed = PreparedAerodynamicCoefficients.from_runtime_tables(lowered)
    static_only = PreparedAerodynamicCoefficients(composed.force_tables, composed.moment_tables)

    common = {
        "velocity_m_s": 17.9,
        "altitude_m": 178.0,
        "collective_elevon": 0.0,
        "differential_elevon": 0.0,
    }
    query = AeroQueryContext.from_air_data(0.08, 0.1, 0.01, common)
    zero_force = composed.context_force_provider()(query)
    zero_moment = composed.context_moment_provider()(query)
    static_force = static_only.context_force_provider()(query)
    static_moment = static_only.context_moment_provider()(query)

    assert zero_force == static_force
    assert zero_moment == static_moment

    commanded = AeroQueryContext.from_air_data(
        0.08,
        0.1,
        0.01,
        {**common, "collective_elevon": 0.0872664626},
    )
    commanded_force = composed.context_force_provider()(commanded)
    commanded_moment = composed.context_moment_provider()(commanded)
    assert commanded_force != zero_force
    assert commanded_moment != zero_moment

    with pytest.raises(ValueError, match="outside"):
        composed.context_force_provider()(AeroQueryContext.from_air_data(0.08, 0.1, 0.01, {**common, "collective_elevon": 1.0}))


def test_x8_surface_authority_derivatives_are_finite_and_source_signed() -> None:
    """The imported X8 control decks expose usable local authority signs."""

    paths = (
        TABLES / "skywalker_x8_static_6axis.tbl",
        TABLES / "skywalker_x8_collective_elevon_6axis.tbl",
        TABLES / "skywalker_x8_differential_elevon_6axis.tbl",
    )
    documents = tuple(TableDocument.model_validate(ingest_file(path).document) for path in paths)
    _, lowered = bind_runtime_tables(documents, {})
    model = PreparedAerodynamicCoefficients.from_runtime_tables(lowered)
    force = model.context_force_provider()
    moment = model.context_moment_provider()
    common = {
        "velocity_m_s": 17.9,
        "altitude_m": 178.0,
        "collective_elevon": math.radians(-2.35),
        "differential_elevon": math.radians(-2.16),
    }
    base = AeroQueryContext.from_air_data(17.9 / 340.0, math.radians(7.9), math.radians(1.2), common)
    delta = math.radians(1.0)
    collective = AeroQueryContext.from_air_data(
        17.9 / 340.0,
        math.radians(7.9),
        math.radians(1.2),
        {**common, "collective_elevon": common["collective_elevon"] + delta},
    )
    differential = AeroQueryContext.from_air_data(
        17.9 / 340.0,
        math.radians(7.9),
        math.radians(1.2),
        {**common, "differential_elevon": common["differential_elevon"] + delta},
    )
    base_force, base_moment = force(base), moment(base)
    collective_force, collective_moment = force(collective), moment(collective)
    differential_force, differential_moment = force(differential), moment(differential)
    values = (
        base_force,
        base_moment,
        collective_force,
        collective_moment,
        differential_force,
        differential_moment,
    )
    assert all(math.isfinite(component) for value in values for component in (value.x, value.y, value.z))
    # These signs are properties of the checked-in source tables, not
    # controller assumptions: positive collective elevon reduces Cm, while
    # positive differential elevon increases CY and Cl.
    assert collective_moment.y < base_moment.y
    assert differential_force.y > base_force.y
    assert differential_moment.x > base_moment.x
    assert differential_moment.z < base_moment.z
####
