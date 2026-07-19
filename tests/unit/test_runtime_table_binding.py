from __future__ import annotations

from pathlib import Path

from taoryx.language.ingest import ingest_file
from taoryx.language.models import TableDocument
from taoryx.runtime.table_binding import bind_runtime_tables

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
    assert "cx" not in available
    assert lowered["cx-collective"].independent_variables[-1] == "collective_elevon"
    assert lowered["cx-differential"].independent_variables[-1] == "differential_elevon"
    assert lowered["cx-static"].independent_variables == ("velocity_m_s", "altitude_m", "alpha", "beta")
####
